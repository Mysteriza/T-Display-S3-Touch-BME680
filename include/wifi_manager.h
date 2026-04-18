#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct WeatherSnapshot
{
    bool valid = false;
    float surface_pressure_hpa = NAN;
    uint32_t last_update_ms = 0;
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

    bool forceFetchNow();
    void printStatus(Stream &out) const;

private:
    WiFiManager() = default;
    WiFiManager(const WiFiManager &) = delete;
    WiFiManager &operator=(const WiFiManager &) = delete;

    enum class State : uint8_t
    {
        BootScan = 0,
        Connecting,
        Online,
        ReconnectAttempts,
        OfflineLocked,
    };

    bool connectWithTimeout(uint32_t timeout_ms);
    bool fetchOpenMeteo();
    bool parseWeatherPayload(const String &payload, WeatherSnapshot &snapshot) const;
    bool parseNumberAfterKey(const String &src, const char *key, float &out_value) const;
    void setRuntimeState(State state, bool connected, bool online_mode, uint8_t reconnect_attempts);

private:
    mutable SemaphoreHandle_t mutex_ = nullptr;

    State state_ = State::BootScan;
    bool connected_ = false;
    bool online_mode_ = false;
    uint8_t reconnect_attempts_ = 0;

    uint32_t state_since_ms_ = 0;
    uint32_t last_fetch_ms_ = 0;
    uint32_t last_reconnect_try_ms_ = 0;

    WeatherSnapshot snapshot_{};
};
