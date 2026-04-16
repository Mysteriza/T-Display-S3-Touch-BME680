#include "sensor_manager.h"

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"

namespace
{
    const uint8_t kBsecIaqConfig[] = {
#include "config/bme680/bme680_iaq_33v_3s_4d/bsec_iaq.txt"
    };

    constexpr uint32_t kDebugIntervalMs = 10000UL;
}

ScopedSemaphoreLock::ScopedSemaphoreLock(SemaphoreHandle_t sem, TickType_t timeout_ticks) : sem_(sem)
{
    if ((sem_ != nullptr) && (xSemaphoreTake(sem_, timeout_ticks) == pdTRUE))
    {
        owns_lock_ = true;
    }
}

ScopedSemaphoreLock::~ScopedSemaphoreLock()
{
    if (owns_lock_ && (sem_ != nullptr))
    {
        xSemaphoreGive(sem_);
    }
}

bool ScopedSemaphoreLock::ownsLock() const
{
    return owns_lock_;
}

SensorManager &SensorManager::instance()
{
    static SensorManager manager;
    return manager;
}

bool SensorManager::isHealthy() const
{
    return healthy_;
}

bool SensorManager::isRealtimeConnected() const
{
    return realtime_connected_;
}

bool SensorManager::detailedDebug() const
{
    return detailed_debug_;
}

void SensorManager::setDetailedDebug(bool enabled)
{
    detailed_debug_ = enabled;
}

float SensorManager::seaLevelPressureHpa() const
{
    return sea_level_hpa_;
}

bool SensorManager::validSeaLevelHpa(float hpa) const
{
    return isfinite(hpa) && (hpa >= cfg::sensor::kSeaLevelMinHpa) && (hpa <= cfg::sensor::kSeaLevelMaxHpa);
}

bool SensorManager::setSeaLevelPressure(float hpa, bool persist)
{
    if (!validSeaLevelHpa(hpa))
    {
        return false;
    }

    {
        ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(50));
        sea_level_hpa_ = hpa;
    }

    if (persist && cfg_ready_)
    {
        cfg_nvs_.putFloat("slp_hpa", sea_level_hpa_);
    }

    refreshAltitudeFromSnapshot();
    return true;
}

bool SensorManager::calibrateSeaLevelFromAltitude(float altitude_m)
{
    if (!isfinite(altitude_m) || (altitude_m < cfg::sensor::kKnownAltitudeMinM) || (altitude_m > cfg::sensor::kKnownAltitudeMaxM))
    {
        return false;
    }

    const SensorData snapshot = getData();
    if (!snapshot.valid || !isfinite(snapshot.pressure_hpa) || (snapshot.pressure_hpa <= 0.0f))
    {
        return false;
    }

    const float base = 1.0f - (altitude_m / 44330.0f);
    if (base <= 0.0f)
    {
        return false;
    }

    const float computed_slp = snapshot.pressure_hpa / powf(base, (1.0f / 0.1903f));
    return setSeaLevelPressure(computed_slp, true);
}

void SensorManager::resetSeaLevelPressure()
{
    setSeaLevelPressure(cfg::sensor::kSeaLevelDefaultHpa, true);
}

float SensorManager::calcAltitude(float pressure_hpa) const
{
    if (!isfinite(pressure_hpa) || (pressure_hpa <= 0.0f) || !isfinite(sea_level_hpa_) || (sea_level_hpa_ <= 0.0f))
    {
        return NAN;
    }
    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa_, 0.1903f));
}

const char *SensorManager::rateName(float rate) const
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

uint32_t SensorManager::rateExpectedIntervalMs(float rate) const
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

uint32_t SensorManager::computeStaleReinitMs(float rate) const
{
    const uint32_t expected_ms = rateExpectedIntervalMs(rate);
    const uint32_t dynamic_ms = expected_ms * 4UL;
    return dynamic_ms > cfg::timing::kSensorStaleReinitMs ? dynamic_ms : cfg::timing::kSensorStaleReinitMs;
}

bool SensorManager::i2cPing(TwoWire &bus, uint8_t address) const
{
    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

bool SensorManager::i2cReadReg(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value) const
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

bool SensorManager::isBme68xChip(TwoWire &bus, uint8_t address, uint8_t *chip_id) const
{
    uint8_t id = 0;
    if (!i2cReadReg(bus, address, cfg::sensor::kBmeChipIdReg, id))
    {
        return false;
    }

    if (chip_id != nullptr)
    {
        *chip_id = id;
    }

    return id == cfg::sensor::kBmeChipIdValue;
}

bool SensorManager::anyBme68xPresentOnBus(TwoWire &bus) const
{
    // Runtime link probing should be lightweight and avoid register reads
    // that can spam requestFrom errors on unstable buses.
    return i2cPing(bus, 0x76) || i2cPing(bus, 0x77);
}

bool SensorManager::anyBme68xPresent() const
{
    if (anyBme68xPresentOnBus(Wire))
    {
        return true;
    }

    if (alt_bus_enabled_ && anyBme68xPresentOnBus(const_cast<TwoWire &>(alt_wire_)))
    {
        return true;
    }

    return false;
}

void SensorManager::appendCandidateIfPresent(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus, uint8_t addr)
{
    if ((count >= list_capacity) || !i2cPing(bus, addr))
    {
        return;
    }

    uint8_t chip_id = 0;
    if (!isBme68xChip(bus, addr, &chip_id))
    {
        if (detailed_debug_)
        {
            Serial.printf("[BME680] skip bus=%s addr=0x%02X chip=0x%02X expected=0x%02X\n",
                          on_alt_bus ? "ALT(43/44)" : "MAIN(18/17)",
                          addr,
                          chip_id,
                          cfg::sensor::kBmeChipIdValue);
        }
        return;
    }

    list[count].wire = &bus;
    list[count].on_alt_bus = on_alt_bus;
    list[count].addr = addr;
    ++count;
}

void SensorManager::appendBusCandidates(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus)
{
    const uint8_t configured_addr = (cfg::pins::kBmeAddressConfigured == 0x77) ? 0x77 : 0x76;
    const uint8_t alternate_addr = (configured_addr == 0x77) ? 0x76 : 0x77;

    appendCandidateIfPresent(list, list_capacity, count, bus, on_alt_bus, configured_addr);
    appendCandidateIfPresent(list, list_capacity, count, bus, on_alt_bus, alternate_addr);
}

void SensorManager::scanI2CBus(TwoWire &bus, char *detected, size_t detected_len, uint8_t *found_count) const
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
        if (!i2cPing(bus, addr))
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

        const size_t remain = detected_len - used;
        used += (static_cast<size_t>(written) < remain) ? static_cast<size_t>(written) : (remain - 1U);
        ++count;
    }

    if (found_count != nullptr)
    {
        *found_count = count;
    }
}

void SensorManager::updateRealtimeLinkStatus(bool present_now)
{
    if (present_now)
    {
        link_fail_streak_ = 0;
        if (link_ok_streak_ < 255U)
        {
            ++link_ok_streak_;
        }
        if (link_ok_streak_ >= cfg::sensor::kLinkOkLimit)
        {
            realtime_connected_ = true;
        }
        return;
    }

    link_ok_streak_ = 0;
    if (link_fail_streak_ < 255U)
    {
        ++link_fail_streak_;
    }

    if (link_fail_streak_ >= cfg::sensor::kLinkFailLimit)
    {
        realtime_connected_ = false;
    }
}

void SensorManager::probeRealtimeLinkIfDue(uint32_t now_ms)
{
    if ((last_link_probe_ms_ != 0U) && (now_ms - last_link_probe_ms_ < cfg::timing::kSensorLinkProbeMs))
    {
        return;
    }

    last_link_probe_ms_ = now_ms;

    bool present_now = false;
    if ((bme_wire_ != nullptr) && ((bme_addr_ == 0x76) || (bme_addr_ == 0x77)))
    {
        present_now = i2cPing(*bme_wire_, bme_addr_);
    }
    else
    {
        present_now = anyBme68xPresent();
    }

    updateRealtimeLinkStatus(present_now);
}

void SensorManager::seedDefaultSnapshot()
{
    sensor_data_ = SensorData{};
    last_safe_snapshot_ = sensor_data_;
    live_ = LiveReadings{};
}

bool SensorManager::ensureStateStoreReady()
{
    if (!state_nvs_ready_)
    {
        state_nvs_ready_ = bsec_nvs_.begin("bsec2", false);
    }
    return state_nvs_ready_;
}

bool SensorManager::restoreBsecState()
{
    if (!ensureStateStoreReady())
    {
        return false;
    }

    if (!bsec_nvs_.isKey("state"))
    {
        return false;
    }

    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    const size_t expected = BSEC_MAX_STATE_BLOB_SIZE;
    const size_t stored = bsec_nvs_.getBytesLength("state");
    if (stored != expected)
    {
        return false;
    }

    const size_t read = bsec_nvs_.getBytes("state", state, expected);
    if (read != expected)
    {
        return false;
    }

    return bsec_.setState(state);
}

bool SensorManager::persistBsecState()
{
    if (!ensureStateStoreReady())
    {
        return false;
    }

    uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};
    if (!bsec_.getState(state))
    {
        return false;
    }

    const size_t saved = bsec_nvs_.putBytes("state", state, BSEC_MAX_STATE_BLOB_SIZE);
    return saved == BSEC_MAX_STATE_BLOB_SIZE;
}

void SensorManager::clearBsecStateBlob()
{
    if (!ensureStateStoreReady())
    {
        return;
    }
    bsec_nvs_.remove("state");
}

void SensorManager::publishSnapshot(uint32_t now_ms)
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(20));
    if (!guard.ownsLock())
    {
        return;
    }

    const bool has_core = isfinite(live_.temperature_c) && isfinite(live_.humidity_pct) && isfinite(live_.pressure_hpa);

    sensor_data_.temperature_c = live_.temperature_c;
    sensor_data_.humidity_pct = live_.humidity_pct;
    sensor_data_.pressure_hpa = live_.pressure_hpa;
    sensor_data_.gas_resistance_kohm = live_.gas_resistance_kohm;
    sensor_data_.iaq = isfinite(live_.iaq) ? live_.iaq : live_.iaq_static;
    sensor_data_.iaq_static = live_.iaq_static;
    sensor_data_.run_in_status = live_.run_in_status;
    sensor_data_.stabilization_status = live_.stabilization_status;
    sensor_data_.iaq_accuracy = live_.iaq_accuracy;
    sensor_data_.altitude_m = calcAltitude(live_.pressure_hpa);
    sensor_data_.valid = has_core;
    sensor_data_.last_update_ms = now_ms;
    last_safe_snapshot_ = sensor_data_;

    live_.has_new_sample = false;
}

void SensorManager::refreshAltitudeFromSnapshot()
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(20));
    if (!guard.ownsLock())
    {
        return;
    }

    if (isfinite(sensor_data_.pressure_hpa) && (sensor_data_.pressure_hpa > 0.0f))
    {
        sensor_data_.altitude_m = calcAltitude(sensor_data_.pressure_hpa);
        sensor_data_.last_update_ms = millis();
    }
}

bool SensorManager::isIaqStuckPattern() const
{
    const bool iaq_flat = isfinite(live_.iaq) &&
                          (fabsf(live_.iaq - cfg::sensor::kIaqStuckTarget) <= cfg::sensor::kIaqStuckTolerance);
    const bool static_flat = isfinite(live_.iaq_static) &&
                             (fabsf(live_.iaq_static - cfg::sensor::kIaqStuckTarget) <= cfg::sensor::kIaqStuckTolerance);

    const bool still_calibrating = (live_.iaq_accuracy <= 1U) &&
                                   (!isfinite(live_.run_in_status) || (live_.run_in_status < 1.0f) ||
                                    !isfinite(live_.stabilization_status) || (live_.stabilization_status < 1.0f));

    return iaq_flat && static_flat && still_calibrating;
}

void SensorManager::maybeRecoverStuckIaq(uint32_t now_ms)
{
    if (iaq_stuck_recovery_done_)
    {
        return;
    }

    if (!isIaqStuckPattern())
    {
        iaq_stuck_since_ms_ = 0;
        return;
    }

    if (iaq_stuck_since_ms_ == 0U)
    {
        iaq_stuck_since_ms_ = now_ms;
        return;
    }

    if (now_ms - iaq_stuck_since_ms_ < cfg::timing::kIaqStuckTimeoutMs)
    {
        return;
    }

    if (detailed_debug_)
    {
        Serial.println("[BME680] IAQ stuck near 50 with low accuracy. Clearing BSEC state and reinitializing once.");
    }

    clearBsecStateBlob();
    iaq_stuck_recovery_done_ = true;
    iaq_stuck_since_ms_ = 0;
    live_ = LiveReadings{};
    healthy_ = false;
    realtime_connected_ = false;
}

bool SensorManager::loadRuntimeConfig()
{
    if (!cfg_ready_)
    {
        cfg_ready_ = cfg_nvs_.begin("sensorcfg", false);
    }

    if (!cfg_ready_)
    {
        sea_level_hpa_ = cfg::sensor::kSeaLevelDefaultHpa;
        return false;
    }

    float loaded = cfg_nvs_.getFloat("slp_hpa", cfg::sensor::kSeaLevelDefaultHpa);
    if (!validSeaLevelHpa(loaded))
    {
        loaded = cfg::sensor::kSeaLevelDefaultHpa;
        cfg_nvs_.putFloat("slp_hpa", loaded);
    }

    sea_level_hpa_ = loaded;
    return true;
}

bool SensorManager::init()
{
    if (sensor_mutex_ == nullptr)
    {
        sensor_mutex_ = xSemaphoreCreateMutex();
    }

    live_ = LiveReadings{};

    healthy_ = false;
    realtime_connected_ = false;
    link_ok_streak_ = 0;
    link_fail_streak_ = 0;
    last_link_probe_ms_ = 0;
    last_saved_accuracy_ = 0;
    iaq_stuck_since_ms_ = 0;

    bme_wire_ = &Wire;
    bme_addr_ = cfg::pins::kBmeAddressConfigured;
    bme_on_alt_bus_ = false;
    active_bsec_rate_ = BSEC_SAMPLE_RATE_LP;

    alt_bus_enabled_ = (cfg::pins::kAltI2cSda != cfg::pins::kMainI2cSda) ||
                       (cfg::pins::kAltI2cScl != cfg::pins::kMainI2cScl);

    loadRuntimeConfig();

    if (!main_bus_ready_)
    {
        Wire.begin();
        main_bus_ready_ = true;
    }
    Wire.setClock(400000);
    Wire.setTimeOut(50);

    if (alt_bus_enabled_)
    {
        if (!alt_bus_ready_)
        {
            alt_wire_.begin(cfg::pins::kAltI2cSda, cfg::pins::kAltI2cScl);
            alt_bus_ready_ = true;
        }
        alt_wire_.setClock(400000);
        alt_wire_.setTimeOut(50);
    }

    BmeCandidate candidates[4]{};
    size_t candidate_count = 0;

    for (uint8_t attempt = 0; (attempt < 6) && (candidate_count == 0); ++attempt)
    {
        candidate_count = 0;
        appendBusCandidates(candidates, 4, candidate_count, Wire, false);
        if (alt_bus_enabled_)
        {
            appendBusCandidates(candidates, 4, candidate_count, alt_wire_, true);
        }

        if (candidate_count == 0)
        {
            delay(120);
        }
    }

    if (candidate_count == 0)
    {
        if (detailed_debug_)
        {
            Serial.println("[BME680] init failed: no sensor detected on main or alt bus.");
        }
        return false;
    }

    bool begin_ok = false;
    for (size_t i = 0; i < candidate_count; ++i)
    {
        bme_wire_ = candidates[i].wire;
        bme_addr_ = candidates[i].addr;
        bme_on_alt_bus_ = candidates[i].on_alt_bus;

        begin_ok = bsec_.begin(bme_addr_, *bme_wire_);
        if (begin_ok)
        {
            break;
        }
    }

    healthy_ = begin_ok;
    realtime_connected_ = begin_ok;
    if (!healthy_)
    {
        return false;
    }

    // Recommended for enclosed products where sensor board self-heating exists.
    bsec_.setTemperatureOffset(1.5f);

    if (!bsec_.setConfig(kBsecIaqConfig))
    {
        healthy_ = false;
        return false;
    }

    restoreBsecState();

    stale_reinit_ms_ = cfg::timing::kSensorStaleReinitMs;

    bsecSensor sensor_list[] = {
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
    };

    const RateCandidate rates[] = {
        {BSEC_SAMPLE_RATE_LP},
        {BSEC_SAMPLE_RATE_SCAN},
    };

    bool subscribed = false;
    for (const RateCandidate &candidate : rates)
    {
        if (bsec_.updateSubscription(sensor_list,
                                     static_cast<uint8_t>(sizeof(sensor_list) / sizeof(sensor_list[0])),
                                     candidate.rate))
        {
            active_bsec_rate_ = candidate.rate;
            stale_reinit_ms_ = computeStaleReinitMs(active_bsec_rate_);
            subscribed = true;
            break;
        }
    }

    healthy_ = subscribed;
    realtime_connected_ = subscribed;
    if (!healthy_)
    {
        return false;
    }

    bsec_.attachCallback(SensorManager::bsecCallbackBridge);

    const uint32_t warmup_start = millis();
    while ((millis() - warmup_start < 4000U) && !live_.has_new_sample)
    {
        bsec_.run();
        delay(40);
    }

    if (live_.has_new_sample)
    {
        const uint32_t now_ms = millis();
        publishSnapshot(now_ms);
        last_publish_ms_ = now_ms;
        last_sample_ms_ = now_ms;
        last_saved_accuracy_ = live_.iaq_accuracy;
    }
    else
    {
        last_publish_ms_ = 0;
        last_sample_ms_ = 0;
    }

    last_state_save_ms_ = millis();
    return true;
}

void SensorManager::bsecCallbackBridge(const bme68xData /*data*/, const bsecOutputs outputs, const Bsec2 /*bsec*/)
{
    SensorManager::instance().onBsecOutputs(outputs);
}

void SensorManager::onBsecOutputs(const bsecOutputs &outputs)
{
    bool has_output = false;

    for (uint8_t i = 0; i < outputs.nOutputs; ++i)
    {
        const bsecData &out = outputs.output[i];
        switch (out.sensor_id)
        {
        case BSEC_OUTPUT_IAQ:
            if (isfinite(out.signal))
            {
                live_.iaq = out.signal;
            }
            live_.iaq_accuracy = out.accuracy;
            has_output = true;
            break;
        case BSEC_OUTPUT_STATIC_IAQ:
            if (isfinite(out.signal))
            {
                live_.iaq_static = out.signal;
            }
            has_output = true;
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
            if (isfinite(out.signal))
            {
                live_.temperature_c = out.signal;
            }
            has_output = true;
            break;
        case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
            if (isfinite(out.signal))
            {
                live_.humidity_pct = out.signal;
            }
            has_output = true;
            break;
        case BSEC_OUTPUT_RAW_PRESSURE:
            // Bosch BSEC2 Arduino wrapper already converts pressure to hPa.
            if (isfinite(out.signal) && (out.signal > 0.0f))
            {
                live_.pressure_hpa = out.signal;
            }
            has_output = true;
            break;
        case BSEC_OUTPUT_RAW_GAS:
            if (isfinite(out.signal) && (out.signal > 0.0f))
            {
                live_.gas_resistance_kohm = out.signal / 1000.0f;
            }
            has_output = true;
            break;
        case BSEC_OUTPUT_RUN_IN_STATUS:
            live_.run_in_status = out.signal;
            has_output = true;
            break;
        case BSEC_OUTPUT_STABILIZATION_STATUS:
            live_.stabilization_status = out.signal;
            has_output = true;
            break;
        default:
            break;
        }
    }

    if (has_output)
    {
        last_sample_ms_ = millis();
        live_.has_new_sample = true;
    }
}

SensorData SensorManager::getData() const
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(30));
    if (guard.ownsLock())
    {
        return sensor_data_;
    }
    return last_safe_snapshot_;
}

void SensorManager::requestReinit()
{
    healthy_ = false;
}

void SensorManager::printStatus(Stream &out) const
{
    const SensorData snapshot = getData();
    const uint32_t now_ms = millis();

    if (snapshot.valid)
    {
        out.printf("[SENSOR] status=%s bus=%s addr=0x%02X mode=%s slp=%.2f hPa age=%lu ms bsec=%d bme=%d\n",
                   healthy_ ? "OK" : "FAIL",
                   bme_on_alt_bus_ ? "ALT(43/44)" : "MAIN(18/17)",
                   bme_addr_,
                   rateName(active_bsec_rate_),
                   sea_level_hpa_,
                   static_cast<unsigned long>(now_ms - snapshot.last_update_ms),
                   static_cast<int>(bsec_.status),
                   static_cast<int>(bsec_.sensor.status));

        out.printf("[SENSOR] data T=%.2f C H=%.2f %% P=%.2f hPa Alt=%.1f m Gas=%.2f kOhm IAQ=%.1f static=%.1f acc=%u runin=%.0f stab=%.0f\n",
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
    else
    {
        out.printf("[SENSOR] status=%s bus=%s addr=0x%02X mode=%s slp=%.2f hPa age=n/a bsec=%d bme=%d\n",
                   healthy_ ? "OK" : "FAIL",
                   bme_on_alt_bus_ ? "ALT(43/44)" : "MAIN(18/17)",
                   bme_addr_,
                   rateName(active_bsec_rate_),
                   sea_level_hpa_,
                   static_cast<int>(bsec_.status),
                   static_cast<int>(bsec_.sensor.status));
    }
}

void SensorManager::printHelp(Stream &out) const
{
    out.println("[CMD] Commands:");
    out.println("[CMD] help                     -> show this help");
    out.println("[CMD] status                   -> current sensor status and data");
    out.println("[CMD] get slp                  -> show sea level pressure (hPa)");
    out.println("[CMD] set slp <hPa>            -> set sea level pressure (850..1100)");
    out.println("[CMD] set alt <meter>          -> compute SLP from current pressure + known altitude");
    out.println("[CMD] reset slp                -> restore default SLP 1013.25 hPa");
    out.println("[CMD] sensor reinit            -> force BME680/BSEC reinit");
    out.println("[CMD] i2c scan                 -> run quick main/alt I2C scan now");
    out.println("[CMD] debug detail on|off      -> force detailed debug output");
}

void SensorManager::scanI2CBuses(Stream &out)
{
    char main_detected[256] = {0};
    uint8_t main_count = 0;
    scanI2CBus(Wire, main_detected, sizeof(main_detected), &main_count);
    out.printf("[CMD] i2c main(%d/%d): %s\n",
               cfg::pins::kMainI2cSda,
               cfg::pins::kMainI2cScl,
               main_count > 0 ? main_detected : "none");

    if (alt_bus_enabled_)
    {
        char alt_detected[256] = {0};
        uint8_t alt_count = 0;
        scanI2CBus(alt_wire_, alt_detected, sizeof(alt_detected), &alt_count);
        out.printf("[CMD] i2c alt(%d/%d): %s\n",
                   cfg::pins::kAltI2cSda,
                   cfg::pins::kAltI2cScl,
                   alt_count > 0 ? alt_detected : "none");
    }
}

void SensorManager::debugTick()
{
    if (!detailed_debug_)
    {
        return;
    }

    const uint32_t now_ms = millis();
    if ((last_debug_ms_ != 0U) && (now_ms - last_debug_ms_ < kDebugIntervalMs))
    {
        return;
    }

    last_debug_ms_ = now_ms;
    printStatus(Serial);
}

void SensorManager::taskEntry(void *parameter)
{
    auto *self = static_cast<SensorManager *>(parameter);
    if (self == nullptr)
    {
        self = &SensorManager::instance();
    }
    self->taskLoop();
}

void SensorManager::taskLoop()
{
    const TickType_t wait_ticks = pdMS_TO_TICKS(cfg::timing::kSensorTaskDelayMs);
    uint32_t last_retry_ms = 0;
    uint8_t run_fail_streak = 0;

    for (;;)
    {
        const uint32_t now_ms = millis();
        probeRealtimeLinkIfDue(now_ms);

        if (!healthy_)
        {
            run_fail_streak = 0;
            if ((last_retry_ms == 0U) || (now_ms - last_retry_ms >= cfg::timing::kSensorInitRetryMs))
            {
                last_retry_ms = now_ms;
                init();
            }
            vTaskDelay(wait_ticks);
            continue;
        }

        if (link_fail_streak_ >= cfg::sensor::kLinkFailLimit)
        {
            realtime_connected_ = false;
        }

        const bool run_ok = bsec_.run();
        if (!run_ok)
        {
            const bool has_hard_error = (bsec_.status < BSEC_OK) || (bsec_.sensor.status < BME68X_OK);
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

            if (run_fail_streak >= cfg::sensor::kRunFailLimit)
            {
                run_fail_streak = 0;
                healthy_ = false;
            }

            vTaskDelay(wait_ticks);
            continue;
        }

        run_fail_streak = 0;

        if (live_.has_new_sample)
        {
            const bool publish_due = (last_publish_ms_ == 0U) ||
                                     (now_ms - last_publish_ms_ >= cfg::timing::kSensorRefreshMs);
            if (publish_due)
            {
                publishSnapshot(now_ms);
                last_publish_ms_ = now_ms;
            }

            const uint8_t latest_accuracy = live_.iaq_accuracy;

            if ((latest_accuracy > last_saved_accuracy_) ||
                (now_ms - last_state_save_ms_ >= cfg::timing::kBsecStateSaveMs))
            {
                if (now_ms - last_state_save_ms_ >= cfg::timing::kBsecStateSaveMinGapMs)
                {
                    if (persistBsecState())
                    {
                        last_state_save_ms_ = now_ms;
                        last_saved_accuracy_ = latest_accuracy;
                    }
                }
            }

            maybeRecoverStuckIaq(now_ms);
            if (!healthy_)
            {
                vTaskDelay(wait_ticks);
                continue;
            }
        }

        if ((last_sample_ms_ != 0U) && (now_ms - last_sample_ms_ > stale_reinit_ms_))
        {
            healthy_ = false;
            vTaskDelay(wait_ticks);
            continue;
        }

        vTaskDelay(wait_ticks);
    }
}

SerialCLI::SerialCLI(SensorManager &sensor_manager) : sensor_manager_(sensor_manager)
{
}

void SerialCLI::trimInPlace(char *text) const
{
    if (text == nullptr)
    {
        return;
    }

    size_t start = 0;
    while ((text[start] != '\0') && isspace(static_cast<unsigned char>(text[start])))
    {
        ++start;
    }

    if (start > 0)
    {
        memmove(text, text + start, strlen(text + start) + 1U);
    }

    size_t len = strlen(text);
    while ((len > 0U) && isspace(static_cast<unsigned char>(text[len - 1U])))
    {
        text[len - 1U] = '\0';
        --len;
    }
}

void SerialCLI::toLowerAscii(char *text) const
{
    if (text == nullptr)
    {
        return;
    }

    for (; *text != '\0'; ++text)
    {
        *text = static_cast<char>(tolower(static_cast<unsigned char>(*text)));
    }
}

void SerialCLI::handleCommand(char *line)
{
    if (line == nullptr)
    {
        return;
    }

    trimInPlace(line);
    if (line[0] == '\0')
    {
        return;
    }

    char original[96] = {0};
    strncpy(original, line, sizeof(original) - 1U);

    char lower[96] = {0};
    strncpy(lower, line, sizeof(lower) - 1U);
    toLowerAscii(lower);

    char *argv[5] = {nullptr, nullptr, nullptr, nullptr, nullptr};
    uint8_t argc = 0;

    char *token = strtok(lower, " \t");
    while ((token != nullptr) && (argc < 5U))
    {
        argv[argc++] = token;
        token = strtok(nullptr, " \t");
    }

    if (argc == 0U)
    {
        return;
    }

    if (strcmp(argv[0], "help") == 0)
    {
        sensor_manager_.printHelp(Serial);
        return;
    }

    if (strcmp(argv[0], "status") == 0)
    {
        sensor_manager_.printStatus(Serial);
        return;
    }

    if ((argc >= 2U) && (strcmp(argv[0], "get") == 0) && (strcmp(argv[1], "slp") == 0))
    {
        Serial.printf("[CMD] sea-level pressure: %.2f hPa\n", sensor_manager_.seaLevelPressureHpa());
        return;
    }

    if ((argc >= 3U) && (strcmp(argv[0], "set") == 0) && (strcmp(argv[1], "slp") == 0))
    {
        const float slp = static_cast<float>(atof(argv[2]));
        if (!sensor_manager_.setSeaLevelPressure(slp, true))
        {
            Serial.printf("[CMD] invalid slp %.2f. valid range: %.1f..%.1f hPa\n",
                          slp,
                          cfg::sensor::kSeaLevelMinHpa,
                          cfg::sensor::kSeaLevelMaxHpa);
            return;
        }

        Serial.printf("[CMD] sea-level pressure set to %.2f hPa\n", sensor_manager_.seaLevelPressureHpa());
        return;
    }

    if ((argc >= 3U) && (strcmp(argv[0], "set") == 0) && (strcmp(argv[1], "alt") == 0))
    {
        const float known_alt_m = static_cast<float>(atof(argv[2]));
        if (!sensor_manager_.calibrateSeaLevelFromAltitude(known_alt_m))
        {
            Serial.printf("[CMD] altitude calibration failed (valid range: %.1f..%.1f m, pressure must be ready).\n",
                          cfg::sensor::kKnownAltitudeMinM,
                          cfg::sensor::kKnownAltitudeMaxM);
            return;
        }

        Serial.printf("[CMD] altitude calibration applied. new slp=%.2f hPa\n", sensor_manager_.seaLevelPressureHpa());
        return;
    }

    if ((argc >= 2U) && (strcmp(argv[0], "reset") == 0) && (strcmp(argv[1], "slp") == 0))
    {
        sensor_manager_.resetSeaLevelPressure();
        Serial.printf("[CMD] sea-level pressure reset to %.2f hPa\n", sensor_manager_.seaLevelPressureHpa());
        return;
    }

    if ((argc >= 2U) && (strcmp(argv[0], "sensor") == 0) && (strcmp(argv[1], "reinit") == 0))
    {
        sensor_manager_.requestReinit();
        Serial.println("[CMD] sensor reinit scheduled.");
        return;
    }

    if ((argc >= 2U) && (strcmp(argv[0], "i2c") == 0) && (strcmp(argv[1], "scan") == 0))
    {
        sensor_manager_.scanI2CBuses(Serial);
        return;
    }

    if ((argc >= 3U) && (strcmp(argv[0], "debug") == 0) && (strcmp(argv[1], "detail") == 0))
    {
        if (strcmp(argv[2], "on") == 0)
        {
            sensor_manager_.setDetailedDebug(true);
            Serial.println("[CMD] detailed debug ON.");
            return;
        }

        if (strcmp(argv[2], "off") == 0)
        {
            sensor_manager_.setDetailedDebug(false);
            Serial.println("[CMD] detailed debug OFF.");
            return;
        }
    }

    Serial.printf("[CMD] unknown command: %s\n", original);
    Serial.println("[CMD] type 'help' for available commands.");
}

void SerialCLI::tick()
{
    while (Serial.available() > 0)
    {
        const int raw = Serial.read();
        if (raw < 0)
        {
            break;
        }

        const char ch = static_cast<char>(raw);
        if ((ch == '\r') || (ch == '\n'))
        {
            if (line_len_ > 0U)
            {
                line_buffer_[line_len_] = '\0';
                handleCommand(line_buffer_);
                line_len_ = 0;
                line_buffer_[0] = '\0';
            }
            continue;
        }

        if (!isprint(static_cast<unsigned char>(ch)))
        {
            continue;
        }

        if (line_len_ < (sizeof(line_buffer_) - 1U))
        {
            line_buffer_[line_len_++] = ch;
            last_rx_ms_ = millis();
        }
        else
        {
            line_len_ = 0;
            line_buffer_[0] = '\0';
            Serial.println("[CMD] input too long, line dropped.");
        }
    }

    if (line_len_ > 0U)
    {
        const uint32_t now_ms = millis();
        if ((last_rx_ms_ != 0U) && (now_ms - last_rx_ms_ >= 600U))
        {
            line_buffer_[line_len_] = '\0';
            handleCommand(line_buffer_);
            line_len_ = 0;
            line_buffer_[0] = '\0';
            last_rx_ms_ = 0;
        }
    }
}
