#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <Wire.h>
#include <bsec2.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct SensorData
{
    float temperature_c = NAN;
    float humidity_pct = NAN;
    float pressure_hpa = NAN;
    float altitude_m = NAN;
    float gas_resistance_kohm = NAN;
    float iaq = NAN;
    float iaq_static = NAN;
    float run_in_status = NAN;
    float stabilization_status = NAN;
    uint8_t iaq_accuracy = 0;
    bool valid = false;
    uint32_t last_update_ms = 0;
};

class ScopedSemaphoreLock
{
public:
    ScopedSemaphoreLock(SemaphoreHandle_t sem, TickType_t timeout_ticks);
    ~ScopedSemaphoreLock();

    bool ownsLock() const;

private:
    SemaphoreHandle_t sem_ = nullptr;
    bool owns_lock_ = false;
};

class SensorManager
{
public:
    static SensorManager &instance();

    bool init();
    bool isHealthy() const;
    bool isRealtimeConnected() const;

    SensorData getData() const;

    void debugTick();
    void taskLoop();
    static void taskEntry(void *parameter);

    void requestReinit();

    void setDetailedDebug(bool enabled);
    bool detailedDebug() const;

    float seaLevelPressureHpa() const;
    bool setSeaLevelPressure(float hpa, bool persist);
    bool calibrateSeaLevelFromAltitude(float altitude_m);
    void resetSeaLevelPressure();

    void printStatus(Stream &out) const;
    void printHelp(Stream &out) const;
    void scanI2CBuses(Stream &out);

private:
    SensorManager() = default;
    SensorManager(const SensorManager &) = delete;
    SensorManager &operator=(const SensorManager &) = delete;

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
    };

    struct BmeCandidate
    {
        TwoWire *wire = nullptr;
        bool on_alt_bus = false;
        uint8_t addr = 0;
    };

    bool loadRuntimeConfig();
    bool ensureStateStoreReady();

    bool restoreBsecState();
    bool persistBsecState();
    void clearBsecStateBlob();

    void seedDefaultSnapshot();
    void publishSnapshot(uint32_t now_ms);
    void refreshAltitudeFromSnapshot();

    float calcAltitude(float pressure_hpa) const;
    const char *rateName(float rate) const;

    uint32_t rateExpectedIntervalMs(float rate) const;
    uint32_t computeStaleReinitMs(float rate) const;

    bool i2cPing(TwoWire &bus, uint8_t address) const;
    bool i2cReadReg(TwoWire &bus, uint8_t address, uint8_t reg, uint8_t &value) const;
    bool isBme68xChip(TwoWire &bus, uint8_t address, uint8_t *chip_id = nullptr) const;
    bool anyBme68xPresentOnBus(TwoWire &bus) const;
    bool anyBme68xPresent() const;

    void appendCandidateIfPresent(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus, uint8_t addr);
    void appendBusCandidates(BmeCandidate *list, size_t list_capacity, size_t &count, TwoWire &bus, bool on_alt_bus);

    void scanI2CBus(TwoWire &bus, char *detected, size_t detected_len, uint8_t *found_count) const;

    void updateRealtimeLinkStatus(bool present_now);
    void probeRealtimeLinkIfDue(uint32_t now_ms);

    bool isIaqStuckPattern() const;
    void maybeRecoverStuckIaq(uint32_t now_ms);

    static void bsecCallbackBridge(const bme68xData data, const bsecOutputs outputs, const Bsec2 bsec);
    void onBsecOutputs(const bsecOutputs &outputs);

    bool validSeaLevelHpa(float hpa) const;

private:
    mutable SemaphoreHandle_t sensor_mutex_ = nullptr;
    SensorData sensor_data_{};
    LiveReadings live_{};

    Bsec2 bsec_;
    TwoWire alt_wire_{1};
    Preferences bsec_nvs_;
    Preferences cfg_nvs_;

    bool healthy_ = false;
    bool realtime_connected_ = false;
    bool alt_bus_enabled_ = false;
    bool main_bus_ready_ = false;
    bool alt_bus_ready_ = false;
    bool detailed_debug_ = false;
    bool cfg_ready_ = false;
    bool state_nvs_ready_ = false;

    float active_bsec_rate_ = BSEC_SAMPLE_RATE_LP;
    float sea_level_hpa_ = 1013.25f;

    TwoWire *bme_wire_ = nullptr;
    uint8_t bme_addr_ = 0x76;
    bool bme_on_alt_bus_ = false;

    uint32_t last_publish_ms_ = 0;
    uint32_t last_sample_ms_ = 0;
    uint32_t last_state_save_ms_ = 0;
    uint32_t last_debug_ms_ = 0;
    uint32_t last_link_probe_ms_ = 0;
    uint32_t stale_reinit_ms_ = 0;
    uint32_t iaq_stuck_since_ms_ = 0;

    uint8_t link_ok_streak_ = 0;
    uint8_t link_fail_streak_ = 0;
    uint8_t last_saved_accuracy_ = 0;

    bool iaq_stuck_recovery_done_ = false;
};

class SerialCLI
{
public:
    explicit SerialCLI(SensorManager &sensor_manager);
    void tick();

private:
    void trimInPlace(char *text) const;
    void toLowerAscii(char *text) const;
    void handleCommand(char *line);

private:
    SensorManager &sensor_manager_;
    char line_buffer_[96] = {0};
    size_t line_len_ = 0;
    uint32_t last_rx_ms_ = 0;
};
