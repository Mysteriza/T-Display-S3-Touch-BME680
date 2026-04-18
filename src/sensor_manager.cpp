#include "sensor_manager.h"

#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "i2c_bus_lock.h"
#include "wifi_manager.h"

namespace
{
    const uint8_t kBsecEnvConfig[] = {
#include "config/bme680/bme680_iaq_33v_3s_4d/bsec_iaq.txt"
    };

    constexpr uint32_t kDebugIntervalMs = 10000UL;
    constexpr uint32_t kGasTrendWindowMs = 5UL * 60UL * 1000UL;
    constexpr uint32_t kGasHistoryPushMs = 2UL * 60UL * 1000UL;
    constexpr float kGasTrendStableDeltaKohm = 0.35f;
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

    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(50));
    if (!guard.ownsLock())
    {
        return false;
    }

    sea_level_hpa_ = hpa;

    if (isfinite(sensor_data_.pressure_hpa) && (sensor_data_.pressure_hpa > 0.0f))
    {
        sensor_data_.altitude_m = calcAltitude(sensor_data_.pressure_hpa);
        sensor_data_.last_update_ms = millis();
        last_safe_snapshot_ = sensor_data_;
    }

    if (persist && cfg_ready_)
    {
        cfg_nvs_.putFloat("slp_hpa", sea_level_hpa_);
    }

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
    ScopedI2cBusLock lock(pdMS_TO_TICKS(40));
    if (!lock.ownsLock())
    {
        return false;
    }

    bus.beginTransmission(address);
    return bus.endTransmission() == 0;
}

bool SensorManager::i2cReadReg(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value) const
{
    ScopedI2cBusLock lock(pdMS_TO_TICKS(40));
    if (!lock.ownsLock())
    {
        return false;
    }

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

bool SensorManager::anyBme68xPresentOnBus(TwoWire &bus)
{
    return i2cPing(bus, 0x76) || i2cPing(bus, 0x77);
}

bool SensorManager::anyBme68xPresent()
{
    if (anyBme68xPresentOnBus(Wire))
    {
        return true;
    }

    if (alt_bus_enabled_ && anyBme68xPresentOnBus(alt_wire_))
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

void SensorManager::updateGasTrend(uint32_t now_ms)
{
    if (!isfinite(live_.gas_resistance_kohm) || (live_.gas_resistance_kohm <= 0.0f))
    {
        return;
    }

    if ((gas_history_last_push_ms_ != 0U) && ((now_ms - gas_history_last_push_ms_) < kGasHistoryPushMs))
    {
        return;
    }

    gas_history_last_push_ms_ = now_ms;

    gas_history_[gas_history_head_].ts_ms = now_ms;
    gas_history_[gas_history_head_].gas_kohm = live_.gas_resistance_kohm;
    gas_history_head_ = (gas_history_head_ + 1U) % kGasHistoryCapacity;
    if (gas_history_count_ < kGasHistoryCapacity)
    {
        ++gas_history_count_;
    }

    float baseline = NAN;
    for (size_t i = 0; i < gas_history_count_; ++i)
    {
        const size_t idx = (gas_history_head_ + kGasHistoryCapacity - 1U - i) % kGasHistoryCapacity;
        const GasHistoryEntry &entry = gas_history_[idx];
        if ((now_ms - entry.ts_ms) >= kGasTrendWindowMs)
        {
            baseline = entry.gas_kohm;
            break;
        }
    }

    if (!isfinite(baseline) || (baseline <= 0.0f))
    {
        sensor_data_.gas_delta_5m_kohm = 0.0f;
        sensor_data_.gas_trend_5m = 0;
        return;
    }

    const float delta = live_.gas_resistance_kohm - baseline;
    sensor_data_.gas_delta_5m_kohm = delta;
    if (delta > kGasTrendStableDeltaKohm)
    {
        sensor_data_.gas_trend_5m = 1;
    }
    else if (delta < -kGasTrendStableDeltaKohm)
    {
        sensor_data_.gas_trend_5m = -1;
    }
    else
    {
        sensor_data_.gas_trend_5m = 0;
    }
}

void SensorManager::publishSnapshot(uint32_t now_ms)
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(20));
    if (!guard.ownsLock())
    {
        return;
    }

    updateGasTrend(now_ms);

    if (isfinite(live_.temperature_c))
    {
        sensor_data_.temperature_c = live_.temperature_c;
    }
    if (isfinite(live_.humidity_pct))
    {
        sensor_data_.humidity_pct = live_.humidity_pct;
    }
    if (isfinite(live_.pressure_hpa) && (live_.pressure_hpa > 0.0f))
    {
        sensor_data_.pressure_hpa = live_.pressure_hpa;
    }
    if (isfinite(live_.gas_resistance_kohm) && (live_.gas_resistance_kohm > 0.0f))
    {
        sensor_data_.gas_resistance_kohm = live_.gas_resistance_kohm;
    }

    const bool has_core = isfinite(sensor_data_.temperature_c) && isfinite(sensor_data_.humidity_pct) &&
                          isfinite(sensor_data_.pressure_hpa) && (sensor_data_.pressure_hpa > 0.0f) &&
                          isfinite(sensor_data_.gas_resistance_kohm) && (sensor_data_.gas_resistance_kohm > 0.0f);

    float effective_pressure_hpa = sensor_data_.pressure_hpa;
    const WeatherSnapshot weather = WiFiManager::instance().getSnapshot();
    if (weather.valid && isfinite(weather.surface_pressure_hpa) &&
        (weather.surface_pressure_hpa >= cfg::sensor::kSeaLevelMinHpa) &&
        (weather.surface_pressure_hpa <= cfg::sensor::kSeaLevelMaxHpa))
    {
        // Blend local pressure and remote surface-pressure reference to reduce short-term noise.
        constexpr float kWeatherPressureBlend = 0.18f;
        effective_pressure_hpa = (sensor_data_.pressure_hpa * (1.0f - kWeatherPressureBlend)) +
                                 (weather.surface_pressure_hpa * kWeatherPressureBlend);
    }

    sensor_data_.altitude_m = calcAltitude(effective_pressure_hpa);
    sensor_data_.valid = has_core;
    if (has_core)
    {
        sensor_data_.last_update_ms = now_ms;
        last_safe_snapshot_ = sensor_data_;
    }

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
        if (sensor_mutex_ == nullptr)
        {
            Serial.println("[SENSOR] init failed: mutex allocation failed.");
            return false;
        }
    }

    live_ = LiveReadings{};
    sensor_data_ = SensorData{};

    healthy_ = false;
    realtime_connected_ = false;
    link_ok_streak_ = 0;
    link_fail_streak_ = 0;
    last_link_probe_ms_ = 0;
    gas_history_head_ = 0;
    gas_history_count_ = 0;
    gas_history_last_push_ms_ = 0;

    bme_wire_ = &Wire;
    bme_addr_ = cfg::pins::kBmeAddressConfigured;
    bme_on_alt_bus_ = false;
    active_bsec_rate_ = BSEC_SAMPLE_RATE_LP;

    alt_bus_enabled_ = (cfg::pins::kAltI2cSda != cfg::pins::kMainI2cSda) ||
                       (cfg::pins::kAltI2cScl != cfg::pins::kMainI2cScl);

    loadRuntimeConfig();

    if (!main_bus_ready_)
    {
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

        ScopedI2cBusLock lock(pdMS_TO_TICKS(120));
        begin_ok = lock.ownsLock() && bsec_.begin(bme_addr_, *bme_wire_);
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

    if (fabsf(cfg::sensor::kBsecTemperatureOffsetC) > 0.01f)
    {
        bsec_.setTemperatureOffset(cfg::sensor::kBsecTemperatureOffsetC);
    }

    {
        ScopedI2cBusLock lock(pdMS_TO_TICKS(120));
        if (!lock.ownsLock() || !bsec_.setConfig(kBsecEnvConfig))
        {
            healthy_ = false;
            return false;
        }
    }

    restoreBsecState();

    stale_reinit_ms_ = cfg::timing::kSensorStaleReinitMs;

    bsecSensor sensor_list[] = {
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_RAW_GAS,
    };

    const float rates[] = {
        BSEC_SAMPLE_RATE_LP,
        BSEC_SAMPLE_RATE_SCAN,
    };

    bool subscribed = false;
    for (float candidate_rate : rates)
    {
        ScopedI2cBusLock lock(pdMS_TO_TICKS(120));
        if (lock.ownsLock() &&
            bsec_.updateSubscription(sensor_list,
                                     static_cast<uint8_t>(sizeof(sensor_list) / sizeof(sensor_list[0])),
                                     candidate_rate))
        {
            active_bsec_rate_ = candidate_rate;
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

    last_publish_ms_ = 0;
    last_sample_ms_.store(0U, std::memory_order_relaxed);
    last_state_save_ms_ = millis();
    return true;
}

void SensorManager::bsecCallbackBridge(const bme68xData /*data*/, const bsecOutputs outputs, const Bsec2 /*bsec*/)
{
    SensorManager::instance().onBsecOutputs(outputs);
}

void SensorManager::onBsecOutputs(const bsecOutputs &outputs)
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(20));
    if (!guard.ownsLock())
    {
        return;
    }

    bool has_output = false;

    for (uint8_t i = 0; i < outputs.nOutputs; ++i)
    {
        const bsecData &out = outputs.output[i];
        switch (out.sensor_id)
        {
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
        default:
            break;
        }
    }

    if (has_output)
    {
        last_sample_ms_.store(millis(), std::memory_order_relaxed);
        live_.has_new_sample = true;
    }
}

SensorData SensorManager::getData() const
{
    ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(30));
    if (guard.ownsLock())
    {
        if (sensor_data_.valid)
        {
            return sensor_data_;
        }
        if (last_safe_snapshot_.valid)
        {
            return last_safe_snapshot_;
        }
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

        out.printf("[SENSOR] data T=%.2f C H=%.2f %% P=%.2f hPa Alt=%.1f m Gas=%.2f kOhm dGas5m=%+.2f trend=%d\n",
                   snapshot.temperature_c,
                   snapshot.humidity_pct,
                   snapshot.pressure_hpa,
                   snapshot.altitude_m,
                   snapshot.gas_resistance_kohm,
                   snapshot.gas_delta_5m_kohm,
                   static_cast<int>(snapshot.gas_trend_5m));
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
    out.println("[CMD] wifi status              -> show WiFi/weather runtime status");
    out.println("[CMD] weather status           -> show weather snapshot and age");
    out.println("[CMD] weather fetch now        -> trigger immediate Open-Meteo fetch");
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

        bool run_ok = false;
        {
            ScopedI2cBusLock lock(pdMS_TO_TICKS(120));
            if (lock.ownsLock())
            {
                run_ok = bsec_.run();
            }
        }

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

        bool has_new_sample = false;
        bool has_core_live = false;
        {
            ScopedSemaphoreLock guard(sensor_mutex_, pdMS_TO_TICKS(20));
            if (guard.ownsLock())
            {
                has_new_sample = live_.has_new_sample;
                has_core_live = isfinite(live_.temperature_c) &&
                                isfinite(live_.humidity_pct) &&
                                isfinite(live_.pressure_hpa) && (live_.pressure_hpa > 0.0f) &&
                                isfinite(live_.gas_resistance_kohm) && (live_.gas_resistance_kohm > 0.0f);
            }
        }

        if (has_new_sample)
        {
            const bool publish_due = has_core_live &&
                                     ((last_publish_ms_ == 0U) ||
                                      (now_ms - last_publish_ms_ >= cfg::timing::kSensorRefreshMs));
            if (publish_due)
            {
                publishSnapshot(now_ms);
                last_publish_ms_ = now_ms;
            }

            if (now_ms - last_state_save_ms_ >= cfg::timing::kBsecStateSaveMs)
            {
                if (now_ms - last_state_save_ms_ >= cfg::timing::kBsecStateSaveMinGapMs)
                {
                    if (persistBsecState())
                    {
                        last_state_save_ms_ = now_ms;
                    }
                }
            }
        }

        const uint32_t last_sample_ms = last_sample_ms_.load(std::memory_order_relaxed);
        if ((last_sample_ms != 0U) && (now_ms - last_sample_ms > stale_reinit_ms_))
        {
            healthy_ = false;
            vTaskDelay(wait_ticks);
            continue;
        }

        vTaskDelay(wait_ticks);
    }
}
