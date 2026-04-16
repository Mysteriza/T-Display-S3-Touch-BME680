#include "sensors.h"

#include <Preferences.h>
#include <Wire.h>
#include <bsec2.h>
#include <math.h>
#include <float.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "config.h"

namespace
{

    Bsec2 g_bsec;
    TwoWire g_alt_wire(1);
    Preferences g_nvs;
    Preferences g_cfg_nvs;
    SemaphoreHandle_t g_sensor_mutex = nullptr;
    SensorData g_sensor_data{};
    volatile bool g_sensor_healthy = false;
    volatile bool g_sensor_connected_realtime = false;
    bool g_alt_bus_enabled = false;
    bool g_force_detailed_logs = false;
    bool g_cfg_ready = false;
    float g_active_bsec_rate = BSEC_SAMPLE_RATE_LP;
    float g_sea_level_hpa = 1013.25f;

    TwoWire *g_bme_wire = &Wire;
    uint8_t g_bme_addr = BME680_I2C_ADDR;
    bool g_bme_on_alt_bus = false;

    uint32_t g_last_publish_ms = 0;
    uint32_t g_last_sample_ms = 0;
    uint32_t g_last_state_save_ms = 0;
    uint32_t g_last_debug_ms = 0;
    uint32_t g_last_link_probe_ms = 0;
    uint32_t g_stale_reinit_ms = SENSOR_REFRESH + 5000UL;
    uint8_t g_link_ok_streak = 0;
    uint8_t g_link_fail_streak = 0;

    constexpr uint32_t I2C_DEBUG_INTERVAL_MS = 10000UL;
    constexpr uint32_t SENSOR_INIT_RETRY_MS = 1000UL;
    constexpr uint32_t SENSOR_STALE_REINIT_MS = SENSOR_REFRESH + 5000UL;
    constexpr uint8_t SENSOR_RUN_FAIL_LIMIT = 2;
    constexpr uint32_t SENSOR_LINK_PROBE_INTERVAL_MS = 500UL;
    constexpr uint8_t SENSOR_LINK_OK_LIMIT = 2;
    constexpr uint8_t SENSOR_LINK_FAIL_LIMIT = 2;
    constexpr uint8_t BME68X_CHIP_ID_REG = 0xD0;
    constexpr uint8_t BME68X_CHIP_ID_VALUE = 0x61;
    constexpr float SEA_LEVEL_DEFAULT_HPA = 1013.25f;
    constexpr float SEA_LEVEL_MIN_HPA = 850.0f;
    constexpr float SEA_LEVEL_MAX_HPA = 1100.0f;
    constexpr float KNOWN_ALT_MIN_M = -500.0f;
    constexpr float KNOWN_ALT_MAX_M = 9000.0f;

    uint8_t g_bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    uint8_t g_bsec_work_buffer[BSEC_MAX_WORKBUFFER_SIZE] = {0};

    struct LiveReadings
    {
        float temperature_c = NAN;
        float humidity_pct = NAN;
        float pressure_hpa = NAN;
        float gas_resistance_kohm = NAN;
        float iaq = NAN;
        float iaq_static = NAN;
        float run_in_status = NAN;
        float stabilization_status = NAN;
        uint8_t iaq_accuracy = 0;
        bool has_new_sample = false;
    } g_live;

    float calc_altitude(float pressure_hpa)
    {
        if (!isfinite(pressure_hpa) || pressure_hpa <= 0.0f || !isfinite(g_sea_level_hpa) || g_sea_level_hpa <= 0.0f)
        {
            return NAN;
        }
        return 44330.0f * (1.0f - powf(pressure_hpa / g_sea_level_hpa, 0.1903f));
    }

    bool valid_sea_level_hpa(float hpa)
    {
        return isfinite(hpa) && (hpa >= SEA_LEVEL_MIN_HPA) && (hpa <= SEA_LEVEL_MAX_HPA);
    }

    bool set_sea_level_hpa(float hpa, bool persist)
    {
        if (!valid_sea_level_hpa(hpa))
        {
            return false;
        }

        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            g_sea_level_hpa = hpa;
            xSemaphoreGive(g_sensor_mutex);
        }
        else
        {
            g_sea_level_hpa = hpa;
        }

        if (persist && g_cfg_ready)
        {
            g_cfg_nvs.putFloat("slp_hpa", g_sea_level_hpa);
        }

        return true;
    }

    bool load_runtime_config()
    {
        if (!g_cfg_ready)
        {
            g_cfg_ready = g_cfg_nvs.begin("sensorcfg", false);
        }

        if (!g_cfg_ready)
        {
            g_sea_level_hpa = SEA_LEVEL_DEFAULT_HPA;
            return false;
        }

        float loaded = g_cfg_nvs.getFloat("slp_hpa", SEA_LEVEL_DEFAULT_HPA);
        if (!valid_sea_level_hpa(loaded))
        {
            loaded = SEA_LEVEL_DEFAULT_HPA;
            g_cfg_nvs.putFloat("slp_hpa", loaded);
        }

        g_sea_level_hpa = loaded;
        return true;
    }

    const char *rate_name(float rate)
    {
        if (fabsf(rate - BSEC_SAMPLE_RATE_CONT) < FLT_EPSILON)
        {
            return "CONT";
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_LP) < 0.0002f)
        {
            return "LP";
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_SCAN) < 0.0002f)
        {
            return "SCAN";
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_ULP) < 0.0002f)
        {
            return "ULP";
        }
        return "CUSTOM";
    }

    uint32_t rate_expected_interval_ms(float rate)
    {
        if (fabsf(rate - BSEC_SAMPLE_RATE_ULP) < 0.0002f)
        {
            return 300000UL;
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_LP) < 0.0002f)
        {
            return 3000UL;
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_SCAN) < 0.0002f)
        {
            return 3000UL;
        }
        if (fabsf(rate - BSEC_SAMPLE_RATE_CONT) < FLT_EPSILON)
        {
            return 1000UL;
        }
        return 10000UL;
    }

    uint32_t compute_stale_reinit_ms(float rate)
    {
        const uint32_t expected_ms = rate_expected_interval_ms(rate);
        const uint32_t dynamic_ms = expected_ms * 4UL;
        return dynamic_ms > SENSOR_STALE_REINIT_MS ? dynamic_ms : SENSOR_STALE_REINIT_MS;
    }

    bool i2c_ping(TwoWire &bus, uint8_t address)
    {
        bus.beginTransmission(address);
        return bus.endTransmission() == 0;
    }

    bool i2c_read_reg(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value)
    {
        bus.beginTransmission(address);
        bus.write(reg);
        if (bus.endTransmission(false) != 0)
        {
            return false;
        }

        if (bus.requestFrom(static_cast<int>(address), 1) != 1)
        {
            return false;
        }

        value = bus.read();
        return true;
    }

    bool is_bme68x_chip(TwoWire &bus, uint8_t address, uint8_t *chip_id = nullptr)
    {
        uint8_t id = 0;
        if (!i2c_read_reg(bus, address, BME68X_CHIP_ID_REG, id))
        {
            return false;
        }

        if (chip_id != nullptr)
        {
            *chip_id = id;
        }

        return id == BME68X_CHIP_ID_VALUE;
    }

    bool any_bme68x_present_on_bus(TwoWire &bus)
    {
        return is_bme68x_chip(bus, 0x76) || is_bme68x_chip(bus, 0x77);
    }

    bool any_bme68x_present()
    {
        if (any_bme68x_present_on_bus(Wire))
        {
            return true;
        }

        if (g_alt_bus_enabled && any_bme68x_present_on_bus(g_alt_wire))
        {
            return true;
        }

        return false;
    }

    void update_realtime_link_status(bool present_now)
    {
        if (present_now)
        {
            g_link_fail_streak = 0;
            if (g_link_ok_streak < 255U)
            {
                ++g_link_ok_streak;
            }

            if (g_link_ok_streak >= SENSOR_LINK_OK_LIMIT)
            {
                g_sensor_connected_realtime = true;
            }
            return;
        }

        g_link_ok_streak = 0;
        if (g_link_fail_streak < 255U)
        {
            ++g_link_fail_streak;
        }

        if (g_link_fail_streak >= SENSOR_LINK_FAIL_LIMIT)
        {
            g_sensor_connected_realtime = false;
        }
    }

    void probe_realtime_link_if_due(uint32_t now_ms)
    {
        if ((g_last_link_probe_ms != 0U) && (now_ms - g_last_link_probe_ms < SENSOR_LINK_PROBE_INTERVAL_MS))
        {
            return;
        }

        g_last_link_probe_ms = now_ms;
        update_realtime_link_status(any_bme68x_present());
    }

    struct BmeCandidate
    {
        TwoWire *wire = nullptr;
        bool on_alt_bus = false;
        uint8_t addr = 0;
    };

    void append_candidate_if_present(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus, uint8_t addr)
    {
        if (count >= list_capacity)
        {
            return;
        }
        if (!i2c_ping(bus, addr))
        {
            return;
        }

        uint8_t chip_id = 0;
        if (!is_bme68x_chip(bus, addr, &chip_id))
        {
            if (g_force_detailed_logs)
            {
                Serial.printf("[BME680] skip bus=%s addr=0x%02X: CHIP_ID=0x%02X (expected 0x%02X)\n",
                              on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                              addr,
                              chip_id,
                              BME68X_CHIP_ID_VALUE);
            }
            return;
        }

        list[count].wire = &bus;
        list[count].on_alt_bus = on_alt_bus;
        list[count].addr = addr;
        ++count;
    }

    void append_bus_candidates(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus)
    {
        const uint8_t cfg_addr = (BME680_I2C_ADDR == 0x77) ? 0x77 : 0x76;
        const uint8_t other_addr = (cfg_addr == 0x77) ? 0x76 : 0x77;

        // Prefer configured address, but always include the other valid BME680 I2C address.
        append_candidate_if_present(list, list_capacity, count, bus, on_alt_bus, cfg_addr);
        append_candidate_if_present(list, list_capacity, count, bus, on_alt_bus, other_addr);
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
        bool has_bsec_output = false;

        for (uint8_t i = 0; i < outputs.nOutputs; ++i)
        {
            const bsecData &out = outputs.output[i];
            switch (out.sensor_id)
            {
            case BSEC_OUTPUT_IAQ:
                g_live.iaq = out.signal;
                g_live.iaq_accuracy = out.accuracy;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_STATIC_IAQ:
                g_live.iaq_static = out.signal;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                g_live.temperature_c = out.signal;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                g_live.humidity_pct = out.signal;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_RAW_PRESSURE:
                // bsec2.cpp already converts pressure from Pa to hPa before processing.
                g_live.pressure_hpa = out.signal;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_RAW_GAS:
                g_live.gas_resistance_kohm = out.signal / 1000.0f;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
                g_live.run_in_status = out.signal;
                has_bsec_output = true;
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
                g_live.stabilization_status = out.signal;
                has_bsec_output = true;
                break;
            default:
                break;
            }
        }

        if (has_bsec_output)
        {
            g_last_sample_ms = millis();
            g_live.has_new_sample = true;
        }
    }

    void seed_default_snapshot()
    {
        g_sensor_data.temperature_c = NAN;
        g_sensor_data.humidity_pct = NAN;
        g_sensor_data.pressure_hpa = NAN;
        g_sensor_data.altitude_m = NAN;
        g_sensor_data.gas_resistance_kohm = NAN;
        g_sensor_data.iaq = NAN;
        g_sensor_data.iaq_static = NAN;
        g_sensor_data.run_in_status = NAN;
        g_sensor_data.stabilization_status = NAN;
        g_sensor_data.iaq_accuracy = 0;
        g_sensor_data.valid = false;
        g_sensor_data.last_update_ms = 0;

        g_live.temperature_c = NAN;
        g_live.humidity_pct = NAN;
        g_live.pressure_hpa = NAN;
        g_live.gas_resistance_kohm = NAN;
        g_live.iaq = NAN;
        g_live.iaq_static = NAN;
        g_live.run_in_status = NAN;
        g_live.stabilization_status = NAN;
        g_live.iaq_accuracy = 0;
        g_live.has_new_sample = false;
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

        const bool has_core_values = isfinite(g_live.temperature_c) &&
                                     isfinite(g_live.humidity_pct) &&
                                     isfinite(g_live.pressure_hpa);

        g_sensor_data.temperature_c = g_live.temperature_c;
        g_sensor_data.humidity_pct = g_live.humidity_pct;
        g_sensor_data.pressure_hpa = g_live.pressure_hpa;
        g_sensor_data.gas_resistance_kohm = g_live.gas_resistance_kohm;
        g_sensor_data.iaq = isfinite(g_live.iaq) ? g_live.iaq : g_live.iaq_static;
        g_sensor_data.iaq_static = g_live.iaq_static;
        g_sensor_data.run_in_status = g_live.run_in_status;
        g_sensor_data.stabilization_status = g_live.stabilization_status;
        g_sensor_data.iaq_accuracy = g_live.iaq_accuracy;
        g_sensor_data.altitude_m = calc_altitude(g_live.pressure_hpa);
        g_sensor_data.valid = has_core_values;
        g_sensor_data.last_update_ms = now_ms;

        g_live.has_new_sample = false;
        xSemaphoreGive(g_sensor_mutex);
    }

    void refresh_altitude_from_snapshot()
    {
        if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) != pdTRUE)
        {
            return;
        }

        if (isfinite(g_sensor_data.pressure_hpa) && g_sensor_data.pressure_hpa > 0.0f)
        {
            g_sensor_data.altitude_m = calc_altitude(g_sensor_data.pressure_hpa);
            g_sensor_data.last_update_ms = millis();
        }

        xSemaphoreGive(g_sensor_mutex);
    }

    void print_status_line(const SensorData &snapshot)
    {
        const uint32_t now_ms = millis();
        const uint32_t age_ms = snapshot.valid ? (now_ms - snapshot.last_update_ms) : 0U;
        if (snapshot.valid)
        {
            Serial.printf("[SENSOR] status=%s bus=%s addr=0x%02X mode=%s slp=%.2f hPa age=%lu ms\n",
                          g_sensor_healthy ? "OK" : "FAIL",
                          g_bme_on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                          g_bme_addr,
                          rate_name(g_active_bsec_rate),
                          g_sea_level_hpa,
                          static_cast<unsigned long>(age_ms));
        }
        else
        {
            Serial.printf("[SENSOR] status=%s bus=%s addr=0x%02X mode=%s slp=%.2f hPa age=n/a\n",
                          g_sensor_healthy ? "OK" : "FAIL",
                          g_bme_on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                          g_bme_addr,
                          rate_name(g_active_bsec_rate),
                          g_sea_level_hpa);
        }

        if (snapshot.valid)
        {
            Serial.printf("[SENSOR] data T=%.2f C H=%.2f %% P=%.2f hPa Alt=%.1f m Gas=%.2f kOhm IAQ=%.1f static=%.1f acc=%u runin=%.0f stab=%.0f\n",
                          snapshot.temperature_c,
                          snapshot.humidity_pct,
                          snapshot.pressure_hpa,
                          snapshot.altitude_m,
                          snapshot.gas_resistance_kohm,
                          snapshot.iaq,
                          snapshot.iaq_static,
                          snapshot.iaq_accuracy,
                          snapshot.run_in_status,
                          snapshot.stabilization_status);
        }
    }

    void serial_help()
    {
        Serial.println("[CMD] Commands:");
        Serial.println("[CMD] help                     -> show this help");
        Serial.println("[CMD] status                   -> current sensor status and data");
        Serial.println("[CMD] get slp                  -> show sea level pressure (hPa)");
        Serial.println("[CMD] set slp <hPa>            -> set sea level pressure (850..1100)");
        Serial.println("[CMD] set alt <meter>          -> compute SLP from current pressure + known altitude");
        Serial.println("[CMD] reset slp                -> restore default SLP 1013.25 hPa");
        Serial.println("[CMD] sensor reinit            -> force BME680/BSEC reinit");
        Serial.println("[CMD] i2c scan                 -> run quick main/alt I2C scan now");
        Serial.println("[CMD] debug detail on|off      -> force detailed debug output");
    }

    void serial_i2c_scan_now()
    {
        char main_detected[256] = {0};
        uint8_t main_found_count = 0;
        scan_i2c_bus(Wire, main_detected, sizeof(main_detected), &main_found_count);
        Serial.printf("[CMD] i2c main(%d/%d): %s\n",
                      PIN_I2C_SDA,
                      PIN_I2C_SCL,
                      main_found_count > 0 ? main_detected : "none");

        if (g_alt_bus_enabled)
        {
            char alt_detected[256] = {0};
            uint8_t alt_found_count = 0;
            scan_i2c_bus(g_alt_wire, alt_detected, sizeof(alt_detected), &alt_found_count);
            Serial.printf("[CMD] i2c alt(%d/%d): %s\n",
                          PIN_I2C_ALT_SDA,
                          PIN_I2C_ALT_SCL,
                          alt_found_count > 0 ? alt_detected : "none");
        }
    }

    void trim_inplace(char *s)
    {
        if (s == nullptr)
        {
            return;
        }

        size_t start = 0;
        while (s[start] != '\0' && isspace(static_cast<unsigned char>(s[start])))
        {
            ++start;
        }

        if (start > 0)
        {
            memmove(s, s + start, strlen(s + start) + 1U);
        }

        size_t len = strlen(s);
        while (len > 0 && isspace(static_cast<unsigned char>(s[len - 1U])))
        {
            s[len - 1U] = '\0';
            --len;
        }
    }

    void to_lower_ascii(char *s)
    {
        if (s == nullptr)
        {
            return;
        }

        for (; *s != '\0'; ++s)
        {
            *s = static_cast<char>(tolower(static_cast<unsigned char>(*s)));
        }
    }

    void handle_serial_command(char *line)
    {
        if (line == nullptr)
        {
            return;
        }

        trim_inplace(line);
        if (line[0] == '\0')
        {
            return;
        }

        char original[96] = {0};
        strncpy(original, line, sizeof(original) - 1U);

        char lower[96] = {0};
        strncpy(lower, line, sizeof(lower) - 1U);
        to_lower_ascii(lower);

        char *argv[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
        uint8_t argc = 0;
        char *token = strtok(lower, " \t");
        while (token != nullptr && argc < 5)
        {
            argv[argc++] = token;
            token = strtok(nullptr, " \t");
        }

        if (argc == 0)
        {
            return;
        }

        if (strcmp(argv[0], "help") == 0)
        {
            serial_help();
            return;
        }

        if (strcmp(argv[0], "status") == 0)
        {
            const SensorData snapshot = sensors_get_data();
            print_status_line(snapshot);
            return;
        }

        if (argc >= 2 && strcmp(argv[0], "get") == 0 && strcmp(argv[1], "slp") == 0)
        {
            Serial.printf("[CMD] sea-level pressure: %.2f hPa\n", g_sea_level_hpa);
            return;
        }

        if (argc >= 3 && strcmp(argv[0], "set") == 0 && strcmp(argv[1], "slp") == 0)
        {
            const float slp = static_cast<float>(atof(argv[2]));
            if (!set_sea_level_hpa(slp, true))
            {
                Serial.printf("[CMD] invalid slp %.2f. valid range: %.1f..%.1f hPa\n", slp, SEA_LEVEL_MIN_HPA, SEA_LEVEL_MAX_HPA);
                return;
            }
            refresh_altitude_from_snapshot();
            Serial.printf("[CMD] sea-level pressure set to %.2f hPa\n", g_sea_level_hpa);
            return;
        }

        if (argc >= 3 && strcmp(argv[0], "set") == 0 && strcmp(argv[1], "alt") == 0)
        {
            const float known_alt_m = static_cast<float>(atof(argv[2]));
            if (!isfinite(known_alt_m) || known_alt_m < KNOWN_ALT_MIN_M || known_alt_m > KNOWN_ALT_MAX_M)
            {
                Serial.printf("[CMD] invalid altitude %.2f m. valid range: %.1f..%.1f m\n", known_alt_m, KNOWN_ALT_MIN_M, KNOWN_ALT_MAX_M);
                return;
            }

            const SensorData snapshot = sensors_get_data();
            if (!snapshot.valid || !isfinite(snapshot.pressure_hpa) || snapshot.pressure_hpa <= 0.0f)
            {
                Serial.println("[CMD] cannot calibrate altitude: pressure data not ready.");
                return;
            }

            const float base = 1.0f - (known_alt_m / 44330.0f);
            if (base <= 0.0f)
            {
                Serial.println("[CMD] invalid altitude for calibration formula.");
                return;
            }

            const float computed_slp = snapshot.pressure_hpa / powf(base, (1.0f / 0.1903f));
            if (!set_sea_level_hpa(computed_slp, true))
            {
                Serial.printf("[CMD] computed SLP %.2f out of range. calibration canceled.\n", computed_slp);
                return;
            }

            refresh_altitude_from_snapshot();

            Serial.printf("[CMD] altitude calibrated: alt=%.2f m pressure=%.2f hPa => slp=%.2f hPa\n",
                          known_alt_m,
                          snapshot.pressure_hpa,
                          g_sea_level_hpa);
            return;
        }

        if (argc >= 2 && strcmp(argv[0], "reset") == 0 && strcmp(argv[1], "slp") == 0)
        {
            set_sea_level_hpa(SEA_LEVEL_DEFAULT_HPA, true);
            refresh_altitude_from_snapshot();
            Serial.printf("[CMD] sea-level pressure reset to %.2f hPa\n", g_sea_level_hpa);
            return;
        }

        if (argc >= 2 && strcmp(argv[0], "sensor") == 0 && strcmp(argv[1], "reinit") == 0)
        {
            g_sensor_healthy = false;
            Serial.println("[CMD] sensor reinit scheduled.");
            return;
        }

        if (argc >= 2 && strcmp(argv[0], "i2c") == 0 && strcmp(argv[1], "scan") == 0)
        {
            serial_i2c_scan_now();
            return;
        }

        if (argc >= 3 && strcmp(argv[0], "debug") == 0 && strcmp(argv[1], "detail") == 0)
        {
            if (strcmp(argv[2], "on") == 0)
            {
                g_force_detailed_logs = true;
                Serial.println("[CMD] detailed debug ON.");
                return;
            }
            if (strcmp(argv[2], "off") == 0)
            {
                g_force_detailed_logs = false;
                Serial.println("[CMD] detailed debug OFF.");
                return;
            }
        }

        Serial.printf("[CMD] unknown command: %s\n", original);
        Serial.println("[CMD] type 'help' for available commands.");
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
    g_sensor_connected_realtime = false;
    g_link_ok_streak = 0;
    g_link_fail_streak = 0;
    g_last_link_probe_ms = 0;
    g_bme_wire = &Wire;
    g_bme_addr = BME680_I2C_ADDR;
    g_bme_on_alt_bus = false;
    g_alt_bus_enabled = (PIN_I2C_ALT_SDA != PIN_I2C_SDA) || (PIN_I2C_ALT_SCL != PIN_I2C_SCL);

    load_runtime_config();

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(50);
    if (g_alt_bus_enabled)
    {
        g_alt_wire.begin(PIN_I2C_ALT_SDA, PIN_I2C_ALT_SCL);
        g_alt_wire.setTimeOut(50);
    }

    BmeCandidate candidates[4]{};
    size_t candidate_count = 0;

    for (uint8_t attempt = 0; attempt < 6 && candidate_count == 0; ++attempt)
    {
        candidate_count = 0;

        append_bus_candidates(candidates, 4, candidate_count, Wire, false);
        if (g_alt_bus_enabled)
        {
            append_bus_candidates(candidates, 4, candidate_count, g_alt_wire, true);
        }

        if (candidate_count == 0)
        {
            delay(120);
        }
    }

    if (candidate_count == 0)
    {
        if (g_force_detailed_logs)
        {
            Serial.println("[BME680] init fail: sensor not found on main or alt I2C bus.");
        }
        return false;
    }

    bool begin_ok = false;
    for (size_t i = 0; i < candidate_count; ++i)
    {
        g_bme_wire = candidates[i].wire;
        g_bme_addr = candidates[i].addr;
        g_bme_on_alt_bus = candidates[i].on_alt_bus;

        if (g_force_detailed_logs)
        {
            Serial.printf("[BME680] init try bus=%s addr=0x%02X\n",
                          g_bme_on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                          g_bme_addr);
        }

        begin_ok = g_bsec.begin(g_bme_addr, *g_bme_wire);
        if (begin_ok)
        {
            break;
        }
    }

    g_sensor_healthy = begin_ok;
    g_sensor_connected_realtime = begin_ok;
    if (!g_sensor_healthy)
    {
        if (g_force_detailed_logs)
        {
            Serial.printf("[BME680] init fail: BSEC begin failed on all candidates (bsec_status=%d sensor_status=%d).\n",
                          static_cast<int>(g_bsec.status),
                          static_cast<int>(g_bsec.sensor.status));
        }
        return false;
    }

    if (g_force_detailed_logs)
    {
        Serial.printf("[BME680] init selected bus=%s addr=0x%02X\n",
                      g_bme_on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                      g_bme_addr);
    }

    g_stale_reinit_ms = SENSOR_STALE_REINIT_MS;

    bsec_virtual_sensor_t sensor_list[] = {
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_RUN_IN_STATUS,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_GAS,
    };

    struct RateCandidate
    {
        float rate;
        const char *name;
    };

    const RateCandidate rate_candidates[] = {
        {BSEC_SAMPLE_RATE_LP, "LP"},
        {BSEC_SAMPLE_RATE_SCAN, "SCAN"},
        {BSEC_SAMPLE_RATE_CONT, "CONT"},
        {BSEC_SAMPLE_RATE_ULP, "ULP"},
    };

    bool subscription_ok = false;
    for (const RateCandidate &candidate : rate_candidates)
    {
        if (g_bsec.updateSubscription(
                sensor_list,
                sizeof(sensor_list) / sizeof(sensor_list[0]),
                candidate.rate))
        {
            g_active_bsec_rate = candidate.rate;
            g_stale_reinit_ms = compute_stale_reinit_ms(g_active_bsec_rate);
            subscription_ok = true;
            if (g_force_detailed_logs)
            {
                Serial.printf("[BME680] subscription mode=%s rate=%.5f OK\n", candidate.name, candidate.rate);
            }
            break;
        }

        if (g_force_detailed_logs)
        {
            Serial.printf("[BME680] subscription mode=%s rate=%.5f fail (bsec_status=%d sensor_status=%d)\n",
                          candidate.name,
                          candidate.rate,
                          static_cast<int>(g_bsec.status),
                          static_cast<int>(g_bsec.sensor.status));
        }
    }

    g_sensor_healthy = subscription_ok;
    g_sensor_connected_realtime = subscription_ok;
    if (!g_sensor_healthy)
    {
        if (g_force_detailed_logs)
        {
            Serial.printf("[BME680] init fail: updateSubscription failed (bsec_status=%d sensor_status=%d).\n",
                          static_cast<int>(g_bsec.status),
                          static_cast<int>(g_bsec.sensor.status));
        }
        return false;
    }

    g_bsec.attachCallback(bsec_callback);

    g_nvs.begin("bsec2", false);
    restore_bsec_state();

    // Try to obtain first valid sample right after successful init so UI can update early.
    const uint32_t warmup_start_ms = millis();
    while ((millis() - warmup_start_ms < 2500U) && !g_live.has_new_sample)
    {
        g_bsec.run();
        delay(40);
    }

    if (g_live.has_new_sample)
    {
        const uint32_t now_ms = millis();
        publish_snapshot(now_ms);
        g_last_publish_ms = now_ms;
        g_last_sample_ms = now_ms;
    }
    else
    {
        g_last_publish_ms = 0;
        g_last_sample_ms = 0;
    }
    g_last_state_save_ms = millis();
    return true;
}

bool sensors_is_healthy()
{
    return g_sensor_healthy;
}

bool sensors_is_connected_realtime()
{
    return g_sensor_connected_realtime;
}

SensorData sensors_get_data()
{
    if (g_sensor_mutex == nullptr)
    {
        seed_default_snapshot();
        return g_sensor_data;
    }

    static SensorData last_copy{};
    if (xSemaphoreTake(g_sensor_mutex, pdMS_TO_TICKS(20)) == pdTRUE)
    {
        last_copy = g_sensor_data;
        xSemaphoreGive(g_sensor_mutex);
    }
    return last_copy;
}

void sensors_debug_tick()
{
    if (!g_force_detailed_logs)
    {
        return;
    }

    const uint32_t now_ms = millis();
    if (now_ms - g_last_debug_ms < I2C_DEBUG_INTERVAL_MS)
    {
        return;
    }
    g_last_debug_ms = now_ms;

    const SensorData snapshot = sensors_get_data();
    const uint32_t sample_age_ms = snapshot.valid ? (now_ms - snapshot.last_update_ms) : UINT32_MAX;
    const bool sample_stale = snapshot.valid && (sample_age_ms > SENSOR_STALE_REINIT_MS);

    if (g_sensor_healthy && snapshot.valid && !sample_stale)
    {
        Serial.printf("[SENSOR] OK bus=%s addr=0x%02X mode=%s age=%lu ms | T=%.2f C H=%.2f %% P=%.2f hPa Alt=%.1f m Gas=%.2f kOhm IAQ=%.1f acc=%u\n",
                      g_bme_on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                      g_bme_addr,
                      rate_name(g_active_bsec_rate),
                      static_cast<unsigned long>(sample_age_ms),
                      snapshot.temperature_c,
                      snapshot.humidity_pct,
                      snapshot.pressure_hpa,
                      snapshot.altitude_m,
                      snapshot.gas_resistance_kohm,
                      snapshot.iaq,
                      snapshot.iaq_accuracy);
        return;
    }

    const bool touch_15 = i2c_ping(Wire, 0x15);
    const bool touch_1a = i2c_ping(Wire, 0x1A);

    const bool bme_main_76 = i2c_ping(Wire, 0x76);
    const bool bme_main_77 = i2c_ping(Wire, 0x77);
    const bool bme_main_cfg = i2c_ping(Wire, BME680_I2C_ADDR);

    uint8_t chip_main_76 = 0;
    uint8_t chip_main_77 = 0;
    const bool chip_main_76_ok = bme_main_76 && is_bme68x_chip(Wire, 0x76, &chip_main_76);
    const bool chip_main_77_ok = bme_main_77 && is_bme68x_chip(Wire, 0x77, &chip_main_77);

    bool bme_alt_76 = false;
    bool bme_alt_77 = false;
    bool bme_alt_cfg = false;
    uint8_t chip_alt_76 = 0;
    uint8_t chip_alt_77 = 0;
    bool chip_alt_76_ok = false;
    bool chip_alt_77_ok = false;
    if (g_alt_bus_enabled)
    {
        bme_alt_76 = i2c_ping(g_alt_wire, 0x76);
        bme_alt_77 = i2c_ping(g_alt_wire, 0x77);
        bme_alt_cfg = i2c_ping(g_alt_wire, BME680_I2C_ADDR);
        chip_alt_76_ok = bme_alt_76 && is_bme68x_chip(g_alt_wire, 0x76, &chip_alt_76);
        chip_alt_77_ok = bme_alt_77 && is_bme68x_chip(g_alt_wire, 0x77, &chip_alt_77);
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

    Serial.printf("[BME680] chip-id main(0x76:%s 0x77:%s) alt(0x76:%s 0x77:%s) expected=0x%02X\n",
                  chip_main_76_ok ? "0x61" : "--",
                  chip_main_77_ok ? "0x61" : "--",
                  chip_alt_76_ok ? "0x61" : "--",
                  chip_alt_77_ok ? "0x61" : "--",
                  BME68X_CHIP_ID_VALUE);

    Serial.printf("[BME680] active bus=%s addr=0x%02X init=%s\n",
                  g_bme_on_alt_bus ? "ALT" : "MAIN",
                  g_bme_addr,
                  g_sensor_healthy ? "OK" : "FAIL");

    if (snapshot.valid)
    {
        Serial.printf("[BME680] mode=%s sample_age=%lu ms\n",
                      rate_name(g_active_bsec_rate),
                      static_cast<unsigned long>(sample_age_ms));
    }
    else
    {
        Serial.printf("[BME680] mode=%s sample_age=n/a\n", rate_name(g_active_bsec_rate));
    }

    const bool any_bme = bme_main_76 || bme_main_77 || bme_alt_76 || bme_alt_77;
    const bool any_bme68x = chip_main_76_ok || chip_main_77_ok || chip_alt_76_ok || chip_alt_77_ok;
    if (!any_bme)
    {
        Serial.println("[BME680] Not detected at 0x76/0x77 on both buses. Check VCC/GND/SDA/SCL, CS->3V3 (I2C mode), and SDO address strap.");
    }
    else if (!any_bme68x)
    {
        Serial.println("[BME680] I2C device found at 0x76/0x77, but CHIP_ID is not 0x61. Device is likely not BME680/BME688.");
    }
    else if (!bme_main_cfg && !bme_alt_cfg)
    {
        const uint8_t detected_addr = bme_alt_77 || bme_main_77 ? 0x77 : 0x76;
        Serial.printf("[BME680] Config addr 0x%02X not present, detected 0x%02X. Firmware now auto-uses detected address.\n",
                      BME680_I2C_ADDR,
                      detected_addr);
    }

    if (snapshot.valid)
    {
        Serial.printf("[BME680] data valid age=%lu ms | T=%.2f C H=%.2f %% P=%.2f hPa Gas=%.2f kOhm IAQ=%.1f acc=%u\n",
                      static_cast<unsigned long>(sample_age_ms),
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
    uint32_t last_retry_ms = 0;
    uint8_t run_fail_streak = 0;

    for (;;)
    {
        const uint32_t now_ms = millis();
        probe_realtime_link_if_due(now_ms);

        if (!g_sensor_healthy)
        {
            run_fail_streak = 0;
            if ((last_retry_ms == 0U) || (now_ms - last_retry_ms >= SENSOR_INIT_RETRY_MS))
            {
                last_retry_ms = now_ms;
                if (g_force_detailed_logs)
                {
                    Serial.println("[BME680] retry init...");
                }
                sensors_init();
            }

            vTaskDelay(wait_ticks);
            continue;
        }

        if (g_sensor_healthy)
        {
            if (g_link_fail_streak >= SENSOR_LINK_FAIL_LIMIT)
            {
                g_sensor_connected_realtime = false;
                g_sensor_healthy = false;
                run_fail_streak = 0;
                vTaskDelay(wait_ticks);
                continue;
            }

            const bool run_ok = g_bsec.run();

            if (!run_ok)
            {
                const bool has_hard_error = (g_bsec.status < BSEC_OK) || (g_bsec.sensor.status < BME68X_OK);
                if (!has_hard_error)
                {
                    run_fail_streak = 0;
                    vTaskDelay(wait_ticks);
                    continue;
                }

                if (run_fail_streak < 255U)
                {
                    ++run_fail_streak;
                }

                if (g_force_detailed_logs)
                {
                    Serial.printf("[BME680] run fail(%u/%u): bsec_status=%d sensor_status=%d\n",
                                  static_cast<unsigned>(run_fail_streak),
                                  static_cast<unsigned>(SENSOR_RUN_FAIL_LIMIT),
                                  static_cast<int>(g_bsec.status),
                                  static_cast<int>(g_bsec.sensor.status));
                }

                if (run_fail_streak < SENSOR_RUN_FAIL_LIMIT)
                {
                    vTaskDelay(wait_ticks);
                    continue;
                }

                if (g_force_detailed_logs)
                {
                    Serial.println("[BME680] run fail limit reached -> reinit scheduled");
                }

                run_fail_streak = 0;
                g_sensor_healthy = false;
                vTaskDelay(wait_ticks);
                continue;
            }

            run_fail_streak = 0;

            // Publish every fresh sample so IAQ/accuracy updates are visible quickly.
            if (g_live.has_new_sample)
            {
                publish_snapshot(now_ms);
                g_last_publish_ms = now_ms;
            }

            if ((g_last_sample_ms != 0U) && (now_ms - g_last_sample_ms > g_stale_reinit_ms))
            {
                run_fail_streak = 0;
                if (g_force_detailed_logs)
                {
                    Serial.printf("[BME680] stale sample (%lu ms > %lu ms), reinit scheduled\n",
                                  static_cast<unsigned long>(now_ms - g_last_sample_ms),
                                  static_cast<unsigned long>(g_stale_reinit_ms));
                }
                g_sensor_healthy = false;
                vTaskDelay(wait_ticks);
                continue;
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

void sensors_serial_tick()
{
    static char line[96] = {0};
    static size_t len = 0;
    static uint32_t last_rx_ms = 0;

    while (Serial.available() > 0)
    {
        const int raw = Serial.read();
        if (raw < 0)
        {
            break;
        }

        const char ch = static_cast<char>(raw);
        if (ch == '\r' || ch == '\n')
        {
            if (len > 0)
            {
                line[len] = '\0';
                handle_serial_command(line);
                len = 0;
                line[0] = '\0';
            }
            continue;
        }

        if (!isprint(static_cast<unsigned char>(ch)))
        {
            continue;
        }

        if (len < (sizeof(line) - 1U))
        {
            line[len++] = ch;
            last_rx_ms = millis();
        }
        else
        {
            len = 0;
            line[0] = '\0';
            Serial.println("[CMD] input too long, line dropped.");
        }
    }

    // Some serial terminals send commands without CR/LF.
    if (len > 0)
    {
        const uint32_t now_ms = millis();
        if ((last_rx_ms != 0U) && (now_ms - last_rx_ms >= 600U))
        {
            line[len] = '\0';
            handle_serial_command(line);
            len = 0;
            line[0] = '\0';
            last_rx_ms = 0;
        }
    }
}