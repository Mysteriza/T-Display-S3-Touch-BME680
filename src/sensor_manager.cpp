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

    constexpr uint8_t kIaqModelBaseline = 0;
    constexpr uint8_t kIaqModelLearning = 1;
    constexpr uint8_t kIaqModelAdaptive = 2;
    constexpr uint8_t kIaqModelFallback = 3;

    struct PersistedIaqAdaptiveState
    {
        uint32_t magic = 0;
        uint16_t version = 0;
        uint16_t reserved = 0;
        float reference_gas_kohm = NAN;
        float adaptive_delta = 0.0f;
        uint32_t adaptive_samples = 0;
        uint8_t model_confidence = 0;
        uint8_t model_state = kIaqModelLearning;
        uint16_t checksum = 0;
    };

    constexpr uint32_t kIaqStateMagic = 0x4941514DU; // 'IAQM'
    constexpr uint16_t kIaqStateVersion = 1;

    uint16_t checksumIaqState(const PersistedIaqAdaptiveState &state)
    {
        const uint8_t *ptr = reinterpret_cast<const uint8_t *>(&state);
        const size_t len = sizeof(PersistedIaqAdaptiveState) - sizeof(state.checksum);
        uint32_t sum = 0;
        for (size_t i = 0; i < len; ++i)
        {
            sum += ptr[i];
        }
        return static_cast<uint16_t>(sum & 0xFFFFU);
    }
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

bool SensorManager::ensureStateStoreReady()
{
    if (!state_nvs_ready_)
    {
        state_nvs_ready_ = bsec_nvs_.begin("bsec2", false);
    }
    return state_nvs_ready_;
}

bool SensorManager::ensureIaqStoreReady()
{
    if (!iaq_store_ready_)
    {
        iaq_store_ready_ = iaq_nvs_.begin("iaqmodel", false);
    }
    return iaq_store_ready_;
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

void SensorManager::loadIaqAdaptiveState()
{
    if (!ensureIaqStoreReady())
    {
        return;
    }

    if (!iaq_nvs_.isKey("state"))
    {
        return;
    }

    PersistedIaqAdaptiveState state{};
    const size_t expected = sizeof(state);
    const size_t stored = iaq_nvs_.getBytesLength("state");
    if (stored != expected)
    {
        return;
    }

    const size_t read = iaq_nvs_.getBytes("state", &state, expected);
    if (read != expected)
    {
        return;
    }

    if ((state.magic != kIaqStateMagic) || (state.version != kIaqStateVersion))
    {
        return;
    }

    if (state.checksum != checksumIaqState(state))
    {
        return;
    }

    if (!isfinite(state.reference_gas_kohm) || (state.reference_gas_kohm <= 0.0f))
    {
        return;
    }

    iaq_reference_gas_kohm_ = state.reference_gas_kohm;
    iaq_adaptive_delta_ = state.adaptive_delta;
    iaq_adaptive_samples_ = state.adaptive_samples;
    iaq_model_confidence_ = state.model_confidence;
    iaq_model_state_ = state.model_state;
}

void SensorManager::saveIaqAdaptiveStateIfDue(uint32_t now_ms, bool force)
{
    if (!ensureIaqStoreReady())
    {
        return;
    }

    if (!force)
    {
        if ((iaq_last_state_save_ms_ != 0U) && (now_ms - iaq_last_state_save_ms_ < cfg::timing::kBsecStateSaveMinGapMs))
        {
            return;
        }
        if ((iaq_last_state_save_ms_ != 0U) && (now_ms - iaq_last_state_save_ms_ < cfg::sensor::kIaqAdaptiveSaveMs))
        {
            return;
        }
    }

    PersistedIaqAdaptiveState state{};
    state.magic = kIaqStateMagic;
    state.version = kIaqStateVersion;
    state.reference_gas_kohm = iaq_reference_gas_kohm_;
    state.adaptive_delta = iaq_adaptive_delta_;
    state.adaptive_samples = iaq_adaptive_samples_;
    state.model_confidence = iaq_model_confidence_;
    state.model_state = iaq_model_state_;
    state.checksum = checksumIaqState(state);

    const size_t saved = iaq_nvs_.putBytes("state", &state, sizeof(state));
    if (saved == sizeof(state))
    {
        iaq_last_state_save_ms_ = now_ms;
    }
}

void SensorManager::appendIaqAdaptiveSample(uint32_t now_ms)
{
    const uint32_t bucket_ms = static_cast<uint32_t>(cfg::sensor::kIaqAdaptiveBucketMinutes) * 60UL * 1000UL;
    if ((iaq_last_bucket_ms_ != 0U) && (now_ms - iaq_last_bucket_ms_ < bucket_ms))
    {
        return;
    }

    if (!isfinite(live_.gas_resistance_kohm) || (live_.gas_resistance_kohm <= 0.0f))
    {
        return;
    }

    float iaq_value = live_.iaq;
    if (!isfinite(iaq_value))
    {
        iaq_value = live_.iaq_static;
    }
    if (!isfinite(iaq_value))
    {
        return;
    }

    iaq_last_bucket_ms_ = now_ms;

    IaqAdaptiveSample sample{};
    sample.ts_ms = now_ms;
    sample.gas_resistance_kohm = live_.gas_resistance_kohm;
    sample.iaq = iaq_value;
    sample.iaq_accuracy = live_.iaq_accuracy;

    iaq_history_[iaq_history_head_] = sample;
    iaq_history_head_ = (iaq_history_head_ + 1U) % kIaqAdaptiveHistoryCapacity;
    if (iaq_history_count_ < kIaqAdaptiveHistoryCapacity)
    {
        ++iaq_history_count_;
    }

    ++iaq_adaptive_samples_;
}

void SensorManager::updateIaqAdaptiveModel(uint32_t now_ms)
{
    appendIaqAdaptiveSample(now_ms);

    float base_iaq = live_.iaq;
    if (!isfinite(base_iaq))
    {
        base_iaq = live_.iaq_static;
    }

    if (!isfinite(base_iaq))
    {
        live_.iaq_effective = NAN;
        live_.iaq_adaptive_delta = 0.0f;
        live_.iaq_model_confidence = 0;
        live_.iaq_model_state = kIaqModelBaseline;
        return;
    }

    if (!isfinite(live_.gas_resistance_kohm) || (live_.gas_resistance_kohm <= 0.0f) || (iaq_history_count_ == 0U))
    {
        live_.iaq_effective = base_iaq;
        live_.iaq_adaptive_delta = 0.0f;
        live_.iaq_model_confidence = 0;
        live_.iaq_model_state = kIaqModelBaseline;
        return;
    }

    float gas_sum = 0.0f;
    uint32_t good_acc_count = 0;
    uint32_t valid_count = 0;
    for (size_t i = 0; i < iaq_history_count_; ++i)
    {
        const IaqAdaptiveSample &sample = iaq_history_[i];
        if (!isfinite(sample.gas_resistance_kohm) || (sample.gas_resistance_kohm <= 0.0f))
        {
            continue;
        }
        gas_sum += sample.gas_resistance_kohm;
        ++valid_count;
        if (sample.iaq_accuracy >= cfg::sensor::kIaqAdaptiveMinAccuracy)
        {
            ++good_acc_count;
        }
    }

    if (valid_count == 0U)
    {
        live_.iaq_effective = base_iaq;
        live_.iaq_adaptive_delta = 0.0f;
        live_.iaq_model_confidence = 0;
        live_.iaq_model_state = kIaqModelBaseline;
        return;
    }

    const float mean_gas = gas_sum / static_cast<float>(valid_count);
    if (!isfinite(mean_gas) || (mean_gas <= 0.0f))
    {
        live_.iaq_effective = base_iaq;
        live_.iaq_adaptive_delta = 0.0f;
        live_.iaq_model_confidence = 0;
        live_.iaq_model_state = kIaqModelBaseline;
        return;
    }

    if (!isfinite(iaq_reference_gas_kohm_) || (iaq_reference_gas_kohm_ <= 0.0f))
    {
        iaq_reference_gas_kohm_ = mean_gas;
    }
    else
    {
        iaq_reference_gas_kohm_ = (iaq_reference_gas_kohm_ * 0.98f) + (mean_gas * 0.02f);
    }

    const float gas_ratio = live_.gas_resistance_kohm / iaq_reference_gas_kohm_;
    float target_delta = 0.0f;
    if (gas_ratio < 1.0f)
    {
        target_delta = (1.0f - gas_ratio) * 40.0f;
    }
    else
    {
        target_delta = -(gas_ratio - 1.0f) * 18.0f;
    }

    if (target_delta > cfg::sensor::kIaqAdaptiveMaxDelta)
    {
        target_delta = cfg::sensor::kIaqAdaptiveMaxDelta;
    }
    if (target_delta < -cfg::sensor::kIaqAdaptiveMaxDelta)
    {
        target_delta = -cfg::sensor::kIaqAdaptiveMaxDelta;
    }

    float target_iaq = static_cast<float>(base_iaq + target_delta);
    if (target_iaq < 0.0f)
    {
        target_iaq = 0.0f;
    }
    if (target_iaq > 500.0f)
    {
        target_iaq = 500.0f;
    }

    const bool run_in_ready = isfinite(live_.run_in_status) && (live_.run_in_status >= cfg::sensor::kIaqAdaptiveRunInReady);
    const bool stabilization_ready = isfinite(live_.stabilization_status) && (live_.stabilization_status >= cfg::sensor::kIaqAdaptiveStabilizationReady);
    const bool history_ready = iaq_history_count_ >= cfg::sensor::kIaqAdaptiveMinSamples;
    const bool adaptation_ready = run_in_ready && stabilization_ready && history_ready &&
                                  (live_.iaq_accuracy >= cfg::sensor::kIaqAdaptiveMinAccuracy);

    if (!adaptation_ready)
    {
        iaq_adaptive_delta_ *= cfg::sensor::kIaqAdaptiveDeltaDecay;
        if (fabsf(iaq_adaptive_delta_) < 0.05f)
        {
            iaq_adaptive_delta_ = 0.0f;
        }

        const float base_err = fabsf(target_iaq - base_iaq);
        iaq_proxy_error_base_ema_ = (iaq_proxy_error_base_ema_ * (1.0f - cfg::sensor::kIaqAdaptiveHealthAlpha)) +
                                    (base_err * cfg::sensor::kIaqAdaptiveHealthAlpha);
        iaq_proxy_error_adaptive_ema_ = iaq_proxy_error_base_ema_;
        iaq_rollback_streak_ = 0;

        const uint32_t max_samples = static_cast<uint32_t>(cfg::sensor::kIaqAdaptiveMinSamples);
        const uint32_t capped_samples = (iaq_history_count_ > max_samples) ? max_samples : static_cast<uint32_t>(iaq_history_count_);
        const uint8_t sample_score = static_cast<uint8_t>((capped_samples * 100U) / max_samples);
        const uint8_t accuracy_score = static_cast<uint8_t>((static_cast<uint16_t>(live_.iaq_accuracy) * 100U) / 3U);
        iaq_model_confidence_ = static_cast<uint8_t>((static_cast<uint16_t>(sample_score) + static_cast<uint16_t>(accuracy_score)) / 2U);
        iaq_model_state_ = kIaqModelLearning;

        live_.iaq_effective = base_iaq;
        live_.iaq_adaptive_delta = 0.0f;
        live_.iaq_model_confidence = iaq_model_confidence_;
        live_.iaq_model_state = iaq_model_state_;

        if (detailed_debug_ && ((iaq_last_digest_ms_ == 0U) || (now_ms - iaq_last_digest_ms_ >= cfg::sensor::kIaqAdaptiveDigestMs)))
        {
            iaq_last_digest_ms_ = now_ms;
            Serial.printf("[IAQ] digest state=learning conf=%u%% acc=%u runin=%.0f stab=%.0f hist=%lu/%lu\n",
                          iaq_model_confidence_,
                          live_.iaq_accuracy,
                          live_.run_in_status,
                          live_.stabilization_status,
                          static_cast<unsigned long>(iaq_history_count_),
                          static_cast<unsigned long>(kIaqAdaptiveHistoryCapacity));
        }

        saveIaqAdaptiveStateIfDue(now_ms);
        return;
    }

    const float next_delta = iaq_adaptive_delta_ + ((target_delta - iaq_adaptive_delta_) * cfg::sensor::kIaqAdaptiveAlpha);
    float delta_step = next_delta - iaq_adaptive_delta_;
    if (delta_step > cfg::sensor::kIaqAdaptiveMaxStep)
    {
        delta_step = cfg::sensor::kIaqAdaptiveMaxStep;
    }
    if (delta_step < -cfg::sensor::kIaqAdaptiveMaxStep)
    {
        delta_step = -cfg::sensor::kIaqAdaptiveMaxStep;
    }
    iaq_adaptive_delta_ += delta_step;

    if (live_.iaq_accuracy < cfg::sensor::kIaqAdaptiveMinAccuracy)
    {
        iaq_adaptive_delta_ *= 0.85f;
        iaq_model_state_ = kIaqModelFallback;
    }

    float effective_iaq = base_iaq + iaq_adaptive_delta_;
    if ((base_iaq >= 200.0f) && (effective_iaq < (base_iaq - cfg::sensor::kIaqAdaptivePoorGuardBand)))
    {
        effective_iaq = base_iaq - cfg::sensor::kIaqAdaptivePoorGuardBand;
    }
    if (effective_iaq < 0.0f)
    {
        effective_iaq = 0.0f;
    }
    if (effective_iaq > 500.0f)
    {
        effective_iaq = 500.0f;
    }

    const float base_err = fabsf(target_iaq - base_iaq);
    const float adaptive_err = fabsf(target_iaq - effective_iaq);
    iaq_proxy_error_base_ema_ = (iaq_proxy_error_base_ema_ * (1.0f - cfg::sensor::kIaqAdaptiveHealthAlpha)) +
                                (base_err * cfg::sensor::kIaqAdaptiveHealthAlpha);
    iaq_proxy_error_adaptive_ema_ = (iaq_proxy_error_adaptive_ema_ * (1.0f - cfg::sensor::kIaqAdaptiveHealthAlpha)) +
                                    (adaptive_err * cfg::sensor::kIaqAdaptiveHealthAlpha);

    const bool degrade_now = (adaptive_err > (base_err + cfg::sensor::kIaqAdaptiveRollbackErrorMargin)) ||
                             (iaq_proxy_error_adaptive_ema_ > (iaq_proxy_error_base_ema_ + cfg::sensor::kIaqAdaptiveRollbackErrorMargin));
    if (degrade_now)
    {
        if (iaq_rollback_streak_ < 255U)
        {
            ++iaq_rollback_streak_;
        }
    }
    else if (iaq_rollback_streak_ > 0U)
    {
        --iaq_rollback_streak_;
    }

    bool rollback_triggered = false;
    if (iaq_rollback_streak_ >= cfg::sensor::kIaqAdaptiveRollbackStreak)
    {
        rollback_triggered = true;
        ++iaq_rollback_count_;
        iaq_rollback_streak_ = 0;
        iaq_adaptive_delta_ = 0.0f;
        effective_iaq = base_iaq;
        iaq_model_state_ = kIaqModelFallback;
    }

    const uint32_t max_samples = static_cast<uint32_t>(cfg::sensor::kIaqAdaptiveMinSamples);
    const uint32_t capped_samples = (iaq_history_count_ > max_samples) ? max_samples : static_cast<uint32_t>(iaq_history_count_);
    const uint8_t sample_score = static_cast<uint8_t>((capped_samples * 100U) / max_samples);
    const uint8_t accuracy_score = static_cast<uint8_t>((good_acc_count * 100U) / valid_count);
    uint8_t health_score = 100U;
    if (iaq_proxy_error_adaptive_ema_ > iaq_proxy_error_base_ema_)
    {
        int32_t h = static_cast<int32_t>(lroundf(100.0f - (iaq_proxy_error_adaptive_ema_ - iaq_proxy_error_base_ema_) * 8.0f));
        if (h < 0)
        {
            h = 0;
        }
        if (h > 100)
        {
            h = 100;
        }
        health_score = static_cast<uint8_t>(h);
    }
    const uint8_t confidence = static_cast<uint8_t>((static_cast<uint16_t>(sample_score) + static_cast<uint16_t>(accuracy_score) + static_cast<uint16_t>(health_score)) / 3U);
    iaq_model_confidence_ = confidence;

    if (iaq_history_count_ < cfg::sensor::kIaqAdaptiveMinSamples)
    {
        iaq_model_state_ = kIaqModelLearning;
    }
    else if ((live_.iaq_accuracy < cfg::sensor::kIaqAdaptiveMinAccuracy) || (confidence < 40U))
    {
        iaq_model_state_ = kIaqModelFallback;
    }
    else
    {
        iaq_model_state_ = kIaqModelAdaptive;
    }
    if (rollback_triggered)
    {
        iaq_model_state_ = kIaqModelFallback;
    }

    live_.iaq_effective = effective_iaq;
    live_.iaq_adaptive_delta = iaq_adaptive_delta_;
    live_.iaq_model_confidence = iaq_model_confidence_;
    live_.iaq_model_state = iaq_model_state_;

    if (detailed_debug_ && ((iaq_last_digest_ms_ == 0U) || (now_ms - iaq_last_digest_ms_ >= cfg::sensor::kIaqAdaptiveDigestMs)))
    {
        iaq_last_digest_ms_ = now_ms;
        Serial.printf("[IAQ] digest state=%u conf=%u%% baseErr=%.2f adaptErr=%.2f emaBase=%.2f emaAdapt=%.2f rb=%lu\n",
                      iaq_model_state_,
                      iaq_model_confidence_,
                      base_err,
                      adaptive_err,
                      iaq_proxy_error_base_ema_,
                      iaq_proxy_error_adaptive_ema_,
                      static_cast<unsigned long>(iaq_rollback_count_));
    }

    saveIaqAdaptiveStateIfDue(now_ms, rollback_triggered);
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
    sensor_data_.iaq_effective = isfinite(live_.iaq_effective) ? live_.iaq_effective : sensor_data_.iaq;
    sensor_data_.iaq_adaptive_delta = live_.iaq_adaptive_delta;
    sensor_data_.iaq_static = live_.iaq_static;
    sensor_data_.run_in_status = live_.run_in_status;
    sensor_data_.stabilization_status = live_.stabilization_status;
    sensor_data_.iaq_accuracy = live_.iaq_accuracy;
    sensor_data_.iaq_model_confidence = live_.iaq_model_confidence;
    sensor_data_.iaq_model_state = live_.iaq_model_state;
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
    loadIaqAdaptiveState();

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

        out.printf("[SENSOR] iaq_model effective=%.1f delta=%+.1f conf=%u%% state=%u\n",
                   snapshot.iaq_effective,
                   snapshot.iaq_adaptive_delta,
                   snapshot.iaq_model_confidence,
                   snapshot.iaq_model_state);
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

void SensorManager::printIaqModelStatus(Stream &out) const
{
    const SensorData snapshot = getData();
    out.printf("[IAQ] model state=%u confidence=%u%% history=%lu/%lu ref_gas=%.2f kOhm delta=%+.2f effective=%.1f\n",
               snapshot.iaq_model_state,
               snapshot.iaq_model_confidence,
               static_cast<unsigned long>(iaq_history_count_),
               static_cast<unsigned long>(kIaqAdaptiveHistoryCapacity),
               iaq_reference_gas_kohm_,
               snapshot.iaq_adaptive_delta,
               snapshot.iaq_effective);
    out.printf("[IAQ] health emaBase=%.2f emaAdapt=%.2f rbStreak=%u rbCount=%lu\n",
               iaq_proxy_error_base_ema_,
               iaq_proxy_error_adaptive_ema_,
               iaq_rollback_streak_,
               static_cast<unsigned long>(iaq_rollback_count_));
}

void SensorManager::resetIaqAdaptiveModel(bool clear_history)
{
    iaq_reference_gas_kohm_ = NAN;
    iaq_adaptive_delta_ = 0.0f;
    iaq_proxy_error_base_ema_ = 0.0f;
    iaq_proxy_error_adaptive_ema_ = 0.0f;
    iaq_adaptive_samples_ = 0;
    iaq_rollback_count_ = 0;
    iaq_model_confidence_ = 0;
    iaq_model_state_ = kIaqModelLearning;
    iaq_last_bucket_ms_ = 0;
    iaq_last_state_save_ms_ = 0;
    iaq_last_digest_ms_ = 0;
    iaq_rollback_streak_ = 0;

    if (clear_history)
    {
        iaq_history_head_ = 0;
        iaq_history_count_ = 0;
        for (size_t i = 0; i < kIaqAdaptiveHistoryCapacity; ++i)
        {
            iaq_history_[i] = IaqAdaptiveSample{};
        }
    }

    live_.iaq_effective = NAN;
    live_.iaq_adaptive_delta = 0.0f;
    live_.iaq_model_confidence = 0;
    live_.iaq_model_state = kIaqModelLearning;

    if (ensureIaqStoreReady())
    {
        iaq_nvs_.remove("state");
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
    out.println("[CMD] iaq model status         -> show adaptive IAQ model diagnostics");
    out.println("[CMD] iaq model digest         -> show concise adaptive health digest");
    out.println("[CMD] iaq model reset          -> clear adaptive IAQ model + history");
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
                updateIaqAdaptiveModel(now_ms);
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
