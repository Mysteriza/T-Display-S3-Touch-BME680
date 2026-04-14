#include "sensors.h"

#include <Preferences.h>
#include <Wire.h>
#include <bsec2.h>
#include <math.h>

#include "config.h"

namespace
{

    Bsec2 g_bsec;
    Preferences g_nvs;
    SemaphoreHandle_t g_sensor_mutex = nullptr;
    SensorData g_sensor_data{};
    bool g_sensor_healthy = false;

    uint32_t g_last_publish_ms = 0;
    uint32_t g_last_state_save_ms = 0;

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

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    g_sensor_healthy = g_bsec.begin(BME680_I2C_ADDR, Wire);
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