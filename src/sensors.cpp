#include "sensors.h"

#include <Preferences.h>
#include <Wire.h>
#include <bsec2.h>
#include <math.h>

#include "config.h"

namespace
{

    Bsec2 g_bsec;
    TwoWire g_alt_wire(1);
    Preferences g_nvs;
    SemaphoreHandle_t g_sensor_mutex = nullptr;
    SensorData g_sensor_data{};
    bool g_sensor_healthy = false;
    bool g_alt_bus_enabled = false;

    TwoWire *g_bme_wire = &Wire;
    uint8_t g_bme_addr = BME680_I2C_ADDR;
    bool g_bme_on_alt_bus = false;

    uint32_t g_last_publish_ms = 0;
    uint32_t g_last_state_save_ms = 0;
    uint32_t g_last_debug_ms = 0;

    constexpr uint32_t I2C_DEBUG_INTERVAL_MS = 5000UL;

    uint8_t g_bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    uint8_t g_bsec_work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};

    struct LiveReadings
    {
        float temperature_c = NAN;
        float humidity_pct = NAN;
        float pressure_hpa = NAN;
        float gas_resistance_kohm = NAN;
        float iaq = NAN;
        uint8_t iaq_accuracy = 0;
        bool has_new_sample = false;
    } g_live;

    float calc_altitude(float pressure_hpa)
    {
        if (!isfinite(pressure_hpa) || pressure_hpa <= 0.0f)
        {
            return NAN;
        }
        return 44330.0f * (1.0f - powf(pressure_hpa / 1013.25f, 0.1903f));
    }

    bool i2c_ping(TwoWire &bus, uint8_t address)
    {
        bus.beginTransmission(address);
        return bus.endTransmission() == 0;
    }

    uint8_t detect_bme_addr(TwoWire &bus)
    {
        const bool has_76 = i2c_ping(bus, 0x76);
        const bool has_77 = i2c_ping(bus, 0x77);

        if (has_76 && has_77)
        {
            return (BME680_I2C_ADDR == 0x77) ? 0x77 : 0x76;
        }
        if (has_76)
        {
            return 0x76;
        }
        if (has_77)
        {
            return 0x77;
        }
        return 0;
    }

    void scan_i2c_bus(TwoWire &bus, char *detected, size_t detected_len, uint8_t *found_count)
    {
        if (detected_len == 0)
        {
            return;
        }

        detected[0] = '\0';
        size_t used = 0;
        uint8_t count = 0;

        for (uint8_t addr = 0x01; addr < 0x7F; ++addr)
        {
            if (!i2c_ping(bus, addr))
            {
                continue;
            }

            const int written = snprintf(detected + used,
                                         detected_len - used,
                                         count == 0 ? "0x%02X" : ",0x%02X",
                                         addr);
            if (written <= 0)
            {
                continue;
            }

            const size_t remaining = detected_len - used;
            if (static_cast<size_t>(written) < remaining)
            {
                used += static_cast<size_t>(written);
            }
            else
            {
                used = detected_len - 1;
            }
            ++count;
        }

        if (found_count != nullptr)
        {
            *found_count = count;
        }
    }

    void bsec_callback(const bme68xData /*raw_data*/, const bsecOutputs outputs, Bsec2 /*bsec*/)
    {
        for (uint8_t i = 0; i < outputs.nOutputs; ++i)
        {
            const bsecData &out = outputs.output[i];
            switch (out.sensor_id)
            {
            case BSEC_OUTPUT_IAQ:
                g_live.iaq = out.signal;
                g_live.iaq_accuracy = out.accuracy;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                g_live.temperature_c = out.signal;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                g_live.humidity_pct = out.signal;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                g_live.pressure_hpa = out.signal / 100.0f;
                break;
            case BSEC_OUTPUT_RAW_GAS:
                g_live.gas_resistance_kohm = out.signal / 1000.0f;
                break;
            default:
                break;
            }
        }
        g_live.has_new_sample = true;
    }

    void seed_default_snapshot()
    {
        g_sensor_data.temperature_c = NAN;
        g_sensor_data.humidity_pct = NAN;
        g_sensor_data.pressure_hpa = NAN;
        g_sensor_data.altitude_m = NAN;
        g_sensor_data.gas_resistance_kohm = NAN;
        g_sensor_data.iaq = NAN;
        g_sensor_data.iaq_accuracy = 0;
        g_sensor_data.valid = false;
        g_sensor_data.last_update_ms = 0;
    }

    bool restore_bsec_state()
    {
        const size_t expected_len = BSEC_MAX_STATE_BLOB_SIZE;
        const size_t stored_len = g_nvs.getBytesLength("state");
        if (stored_len != expected_len)
        {
            return false;
        }

        const size_t read_len = g_nvs.getBytes("state", g_bsec_state, expected_len);
        if (read_len != expected_len)
        {
            return false;
        }

        return g_bsec.setState(g_bsec_state);
    }

    bool persist_bsec_state()
    {
        const bool got_state = g_bsec.getState(g_bsec_state);

        if (!got_state)
        {
            return false;
        }

        const size_t saved = g_nvs.putBytes("state", g_bsec_state, BSEC_MAX_STATE_BLOB_SIZE);
        return saved == BSEC_MAX_STATE_BLOB_SIZE;
    }

    void publish_snapshot(uint32_t now_ms)
    {
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        {
            return;
        }

        g_sensor_data.temperature_c = g_live.temperature_c;
        g_sensor_data.humidity_pct = g_live.humidity_pct;
        g_sensor_data.pressure_hpa = g_live.pressure_hpa;
        g_sensor_data.gas_resistance_kohm = g_live.gas_resistance_kohm;
        g_sensor_data.iaq = g_live.iaq;
        g_sensor_data.iaq_accuracy = g_live.iaq_accuracy;
        g_sensor_data.altitude_m = calc_altitude(g_live.pressure_hpa);
        g_sensor_data.valid = g_live.has_new_sample;
        g_sensor_data.last_update_ms = now_ms;

        g_live.has_new_sample = false;
        xSemaphoreGive(g_sensor_mutex);
    }

} // namespace

bool sensors_init()
{
    if (g_sensor_mutex == nullptr)
    {
        g_sensor_mutex = xSemaphoreCreateMutex();
    }

    seed_default_snapshot();

    g_sensor_healthy = false;
    g_bme_wire = &Wire;
    g_bme_addr = BME680_I2C_ADDR;
    g_bme_on_alt_bus = false;
    g_alt_bus_enabled = (PIN_I2C_ALT_SDA != PIN_I2C_SDA) || (PIN_I2C_ALT_SCL != PIN_I2C_SCL);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    if (g_alt_bus_enabled)
    {
        g_alt_wire.begin(PIN_I2C_ALT_SDA, PIN_I2C_ALT_SCL);
    }

    const uint8_t main_addr = detect_bme_addr(Wire);
    uint8_t alt_addr = 0;
    if (main_addr != 0)
    {
        g_bme_wire = &Wire;
        g_bme_addr = main_addr;
        g_bme_on_alt_bus = false;
    }
    else if (g_alt_bus_enabled)
    {
        alt_addr = detect_bme_addr(g_alt_wire);
        if (alt_addr != 0)
        {
            g_bme_wire = &g_alt_wire;
            g_bme_addr = alt_addr;
            g_bme_on_alt_bus = true;
        }
    }

    if ((main_addr == 0) && (alt_addr == 0))
    {
        Serial.println("[BME680] init fail: sensor not found on main or alt I2C bus.");
        return false;
    }

    Serial.printf("[BME680] init probe selected bus=%s addr=0x%02X\n",
                  g_bme_on_alt_bus ? "ALT(44/43)" : "MAIN(18/17)",
                  g_bme_addr);

    g_sensor_healthy = g_bsec.begin(g_bme_addr, *g_bme_wire);
    if (!g_sensor_healthy)
    {
        return false;
    }

    bsec_virtual_sensor_t sensor_list[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_GAS,
    };

    g_sensor_healthy = g_bsec.updateSubscription(
        sensor_list,
        sizeof(sensor_list) / sizeof(sensor_list[0]),
        BSEC_SAMPLE_RATE_LP);
    if (!g_sensor_healthy)
    {
        return false;
    }

    g_bsec.attachCallback(bsec_callback);

    g_nvs.begin("bsec2", false);
    restore_bsec_state();

    g_last_publish_ms = 0;
    g_last_state_save_ms = millis();
    return true;
}

bool sensors_is_healthy()
{
    return g_sensor_healthy;
}

SensorData sensors_get_data()
{
    if (g_sensor_mutex == nullptr)
    {
        seed_default_snapshot();
        return g_sensor_data;
    }

    SensorData copy{};
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        copy = g_sensor_data;
        xSemaphoreGive(g_sensor_mutex);
    }
    return copy;
}

void sensors_debug_tick()
{
    const uint32_t now_ms = millis();
    if (now_ms - g_last_debug_ms < I2C_DEBUG_INTERVAL_MS)
    {
        return;
    }
    g_last_debug_ms = now_ms;

    const bool touch_15 = i2c_ping(Wire, 0x15);
    const bool touch_1a = i2c_ping(Wire, 0x1A);

    const bool bme_main_76 = i2c_ping(Wire, 0x76);
    const bool bme_main_77 = i2c_ping(Wire, 0x77);
    const bool bme_main_cfg = i2c_ping(Wire, BME680_I2C_ADDR);

    bool bme_alt_76 = false;
    bool bme_alt_77 = false;
    bool bme_alt_cfg = false;
    if (g_alt_bus_enabled)
    {
        bme_alt_76 = i2c_ping(g_alt_wire, 0x76);
        bme_alt_77 = i2c_ping(g_alt_wire, 0x77);
        bme_alt_cfg = i2c_ping(g_alt_wire, BME680_I2C_ADDR);
    }

    char main_detected[256] = {0};
    uint8_t main_found_count = 0;
    scan_i2c_bus(Wire, main_detected, sizeof(main_detected), &main_found_count);

    Serial.printf("[I2C] main(%d/%d) scan count=%u addr=%s\n",
                  PIN_I2C_SDA,
                  PIN_I2C_SCL,
                  main_found_count,
                  main_found_count > 0 ? main_detected : "none");

    if (g_alt_bus_enabled)
    {
        char alt_detected[256] = {0};
        uint8_t alt_found_count = 0;
        scan_i2c_bus(g_alt_wire, alt_detected, sizeof(alt_detected), &alt_found_count);
        Serial.printf("[I2C] alt(%d/%d)  scan count=%u addr=%s\n",
                      PIN_I2C_ALT_SDA,
                      PIN_I2C_ALT_SCL,
                      alt_found_count,
                      alt_found_count > 0 ? alt_detected : "none");
    }

    Serial.printf("[I2C] touch(main:0x15:%s 0x1A:%s) bme(main:0x76:%s 0x77:%s cfg=0x%02X:%s) bme(alt:0x76:%s 0x77:%s cfg=0x%02X:%s)\n",
                  touch_15 ? "OK" : "--",
                  touch_1a ? "OK" : "--",
                  bme_main_76 ? "OK" : "--",
                  bme_main_77 ? "OK" : "--",
                  BME680_I2C_ADDR,
                  bme_main_cfg ? "OK" : "--",
                  bme_alt_76 ? "OK" : "--",
                  bme_alt_77 ? "OK" : "--",
                  BME680_I2C_ADDR,
                  bme_alt_cfg ? "OK" : "--");

    Serial.printf("[BME680] active bus=%s addr=0x%02X init=%s\n",
                  g_bme_on_alt_bus ? "ALT" : "MAIN",
                  g_bme_addr,
                  g_sensor_healthy ? "OK" : "FAIL");

    const bool any_bme = bme_main_76 || bme_main_77 || bme_alt_76 || bme_alt_77;
    if (!any_bme)
    {
        Serial.println("[BME680] Not detected at 0x76/0x77 on both buses. Check VCC/GND/SDA/SCL, CS->3V3 (I2C mode), and SDO address strap.");
    }
    else if (!bme_main_cfg && !bme_alt_cfg)
    {
        Serial.printf("[BME680] Sensor found but config uses 0x%02X. Update BME680_I2C_ADDR in include/config.h.\n", BME680_I2C_ADDR);
    }

    const SensorData snapshot = sensors_get_data();
    if (snapshot.valid)
    {
        const uint32_t age_ms = now_ms - snapshot.last_update_ms;
        Serial.printf("[BME680] data valid age=%lu ms | T=%.2f C H=%.2f %% P=%.2f hPa Gas=%.2f kOhm IAQ=%.1f acc=%u\n",
                      static_cast<unsigned long>(age_ms),
                      snapshot.temperature_c,
                      snapshot.humidity_pct,
                      snapshot.pressure_hpa,
                      snapshot.gas_resistance_kohm,
                      snapshot.iaq,
                      snapshot.iaq_accuracy);
    }
    else
    {
        Serial.printf("[BME680] data not ready. sensors_init=%s\n", g_sensor_healthy ? "OK" : "FAIL");
    }
}

void sensorTask(void * /*parameter*/)
{
    const TickType_t wait_ticks = pdMS_TO_TICKS(200);

    for (;;)
    {
        if (g_sensor_healthy)
        {
            g_bsec.run();
            const uint32_t now_ms = millis();

            if (now_ms - g_last_publish_ms >= SENSOR_REFRESH)
            {
                publish_snapshot(now_ms);
                g_last_publish_ms = now_ms;
            }

            if (now_ms - g_last_state_save_ms >= BSEC_STATE_SAVE_MS)
            {
                persist_bsec_state();
                g_last_state_save_ms = now_ms;
            }
        }

        vTaskDelay(wait_ticks);
    }
}