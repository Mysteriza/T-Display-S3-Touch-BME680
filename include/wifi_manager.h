#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct WeatherSnapshot
{
    bool valid = false;
    float temperature_c = NAN;
    float humidity_pct = NAN;
    float precipitation_mm = 0.0f;
    uint8_t cloud_coverage_pct = 0;
    uint8_t weather_code = 0;
    char weather_desc[32] = {0};
    char local_datetime[24] = {0};
    char forecast_time[12] = {0};
    uint32_t last_update_ms = 0;
    uint32_t last_update_epoch_utc = 0;
};

class WiFiManager
{
public:
    static WiFiManager &instance();

    void init();
    void taskLoop();
    static void taskEntry(void *parameter);

    bool isConnected() const;
    bool isOnlineMode() const;
    bool hasFreshWeather(uint32_t now_ms) const;
    uint32_t lastUpdateAgeMs(uint32_t now_ms) const;
    WeatherSnapshot getSnapshot() const;

    bool isInitialized() const { return mutex_ != nullptr; }

    bool forceFetchNow();
    const char *lastErrorText() const;
    void printStatus(Stream &out) const;

private:
    enum class State : uint8_t
    {
        BootScan = 0,
        Connecting,
        Online,
        ReconnectAttempts,
        OfflineLocked,
    };

    enum class FetchError : uint8_t
    {
        None = 0,
        NotConnected,
        HttpBeginFailed,
        HttpStatusFailed,
        EmptyPayload,
        ParseFailed,
        PressureOutOfRange,
        MutexTimeout,
    };

    int32_t clampI32(int32_t value, int32_t low, int32_t high) const;
    bool connectWithTimeout(uint32_t timeout_ms);
    bool fetchBmkg();
    bool parseWeatherPayload(const String &payload, WeatherSnapshot &snapshot) const;
    bool parseNumberAfterKey(const String &src, const char *key, float &out_value) const;
    void ensureTimeSync();
    void setRuntimeState(State state, bool connected, bool online_mode, uint8_t reconnect_attempts);
    void setFetchError(FetchError error, int http_code = 0);

private:
    mutable SemaphoreHandle_t mutex_ = nullptr;

    State state_ = State::BootScan;
    bool connected_ = false;
    bool online_mode_ = false;
    uint8_t reconnect_attempts_ = 0;

    uint32_t state_since_ms_ = 0;
    uint32_t last_fetch_ms_ = 0;
    uint32_t last_reconnect_try_ms_ = 0;
    uint32_t last_ntp_sync_try_ms_ = 0;
    bool ntp_started_ = false;
    FetchError last_fetch_error_ = FetchError::None;
    int last_http_status_code_ = 0;

    WeatherSnapshot snapshot_{};
};
