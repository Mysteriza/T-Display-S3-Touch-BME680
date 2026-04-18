#include "wifi_manager.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <math.h>

#include "config.h"

WiFiManager &WiFiManager::instance()
{
    static WiFiManager manager;
    return manager;
}

void WiFiManager::init()
{
    if (mutex_ == nullptr)
    {
        mutex_ = xSemaphoreCreateMutex();
    }

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.setSleep(false);
    WiFi.disconnect(false, false);

    const uint32_t now_ms = millis();
    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
    {
        state_ = State::BootScan;
        connected_ = false;
        online_mode_ = false;
        reconnect_attempts_ = 0;
        state_since_ms_ = now_ms;
        last_fetch_ms_ = 0;
        last_reconnect_try_ms_ = 0;
        snapshot_ = WeatherSnapshot{};
        xSemaphoreGive(mutex_);
    }
}

bool WiFiManager::connectWithTimeout(uint32_t timeout_ms)
{
    if ((cfg::wifi::kSsid == nullptr) || (cfg::wifi::kSsid[0] == '\0'))
    {
        return false;
    }

    if ((WiFi.status() == WL_CONNECTED) && (WiFi.SSID() == String(cfg::wifi::kSsid)))
    {
        return true;
    }

    WiFi.disconnect(false, false);
    delay(60);
    WiFi.begin(cfg::wifi::kSsid, cfg::wifi::kPassword);
    const uint32_t start_ms = millis();

    while (millis() - start_ms < timeout_ms)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            return true;
        }
        delay(120);
    }

    return WiFi.status() == WL_CONNECTED;
}

bool WiFiManager::parseNumberAfterKey(const String &src, const char *key, float &out_value) const
{
    const int key_pos = src.indexOf(key);
    if (key_pos < 0)
    {
        return false;
    }

    int colon = src.indexOf(':', key_pos);
    if (colon < 0)
    {
        return false;
    }

    ++colon;
    while ((colon < src.length()) && ((src[colon] == ' ') || (src[colon] == '\t') || (src[colon] == '"')))
    {
        ++colon;
    }

    int end = colon;
    while ((end < src.length()) && ((src[end] == '-') || (src[end] == '+') || (src[end] == '.') || (src[end] >= '0' && src[end] <= '9')))
    {
        ++end;
    }

    if (end <= colon)
    {
        return false;
    }

    out_value = src.substring(colon, end).toFloat();
    return isfinite(out_value);
}

bool WiFiManager::parseWeatherPayload(const String &payload, WeatherSnapshot &snapshot) const
{
    float pressure = NAN;

    const bool ok_pressure = parseNumberAfterKey(payload, "\"surface_pressure\"", pressure);

    if (!ok_pressure)
    {
        return false;
    }

    snapshot.valid = true;
    snapshot.surface_pressure_hpa = pressure;
    snapshot.last_update_ms = millis();
    return true;
}

bool WiFiManager::fetchOpenMeteo()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return false;
    }

    String url;
    url.reserve(256);
    url += "https://api.open-meteo.com/v1/forecast?latitude=";
    url += String(cfg::wifi::kLatitudeDefault, 4);
    url += "&longitude=";
    url += String(cfg::wifi::kLongitudeDefault, 4);
    url += "&current=surface_pressure";

    WiFiClientSecure secure_client;
    secure_client.setInsecure();

    HTTPClient http;
    http.setTimeout(static_cast<uint16_t>(cfg::wifi::kWeatherHttpTimeoutMs));
    if (!http.begin(secure_client, url))
    {
        return false;
    }

    const int code = http.GET();
    if (code != HTTP_CODE_OK)
    {
        http.end();
        return false;
    }

    const String payload = http.getString();
    http.end();

    WeatherSnapshot snapshot{};
    if (!parseWeatherPayload(payload, snapshot))
    {
        return false;
    }

    if ((snapshot.surface_pressure_hpa < cfg::sensor::kSeaLevelMinHpa) || (snapshot.surface_pressure_hpa > cfg::sensor::kSeaLevelMaxHpa))
    {
        return false;
    }

    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) != pdTRUE))
    {
        return false;
    }

    snapshot_ = snapshot;
    last_fetch_ms_ = snapshot.last_update_ms;
    xSemaphoreGive(mutex_);
    return true;
}

bool WiFiManager::isConnected() const
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(30)) != pdTRUE))
    {
        return WiFi.status() == WL_CONNECTED;
    }

    const bool state_connected = connected_;
    xSemaphoreGive(mutex_);
    return state_connected && (WiFi.status() == WL_CONNECTED);
}

bool WiFiManager::isOnlineMode() const
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(30)) != pdTRUE))
    {
        return false;
    }

    const bool mode = online_mode_;
    xSemaphoreGive(mutex_);
    return mode;
}

bool WiFiManager::hasFreshWeather(uint32_t now_ms) const
{
    const uint32_t age = lastUpdateAgeMs(now_ms);
    if (age == UINT32_MAX)
    {
        return false;
    }
    return age <= (cfg::wifi::kWeatherRefreshMs * 2UL);
}

uint32_t WiFiManager::lastUpdateAgeMs(uint32_t now_ms) const
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(30)) != pdTRUE))
    {
        return UINT32_MAX;
    }

    if (!snapshot_.valid || (snapshot_.last_update_ms == 0U))
    {
        xSemaphoreGive(mutex_);
        return UINT32_MAX;
    }

    const uint32_t age = now_ms - snapshot_.last_update_ms;
    xSemaphoreGive(mutex_);
    return age;
}

WeatherSnapshot WiFiManager::getSnapshot() const
{
    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(30)) == pdTRUE))
    {
        const WeatherSnapshot snapshot = snapshot_;
        xSemaphoreGive(mutex_);
        return snapshot;
    }
    return WeatherSnapshot{};
}

bool WiFiManager::forceFetchNow()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        return false;
    }

    const bool ok = fetchOpenMeteo();
    setRuntimeState(ok ? State::Online : State::ReconnectAttempts,
                    WiFi.status() == WL_CONNECTED,
                    ok,
                    reconnect_attempts_);
    return ok;
}

void WiFiManager::setRuntimeState(State state, bool connected, bool online_mode, uint8_t reconnect_attempts)
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) != pdTRUE))
    {
        return;
    }

    state_ = state;
    connected_ = connected;
    online_mode_ = online_mode;
    reconnect_attempts_ = reconnect_attempts;
    state_since_ms_ = millis();
    xSemaphoreGive(mutex_);
}

void WiFiManager::printStatus(Stream &out) const
{
    const WeatherSnapshot snapshot = getSnapshot();
    State state = State::OfflineLocked;
    bool connected = false;
    bool online = false;
    uint8_t attempts = 0;
    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
    {
        state = state_;
        connected = connected_;
        online = online_mode_;
        attempts = reconnect_attempts_;
        xSemaphoreGive(mutex_);
    }

    const char *state_name = "unknown";
    switch (state)
    {
    case State::BootScan:
        state_name = "boot-scan";
        break;
    case State::Connecting:
        state_name = "connecting";
        break;
    case State::Online:
        state_name = "online";
        break;
    case State::ReconnectAttempts:
        state_name = "reconnect";
        break;
    case State::OfflineLocked:
        state_name = "offline";
        break;
    }

    out.printf("[WIFI] state=%s connected=%s online=%s ip=%s attempts=%u\n",
               state_name,
               connected ? "yes" : "no",
               online ? "yes" : "no",
               (connected && (WiFi.status() == WL_CONNECTED)) ? WiFi.localIP().toString().c_str() : "-",
               static_cast<unsigned>(attempts));

    if (snapshot.valid)
    {
        const uint32_t age_ms = (snapshot.last_update_ms == 0U) ? UINT32_MAX : (millis() - snapshot.last_update_ms);
        out.printf("[WEATHER] surface_pressure=%.2f hPa age=%lu ms\n",
                   snapshot.surface_pressure_hpa,
                   static_cast<unsigned long>(age_ms));
    }
    else
    {
        out.println("[WEATHER] no data");
    }
}

void WiFiManager::taskEntry(void *parameter)
{
    auto *self = static_cast<WiFiManager *>(parameter);
    if (self == nullptr)
    {
        self = &WiFiManager::instance();
    }
    self->taskLoop();
}

void WiFiManager::taskLoop()
{
    const TickType_t wait_ticks = pdMS_TO_TICKS(cfg::wifi::kTaskDelayMs);

    for (;;)
    {
        const uint32_t now_ms = millis();
        bool connected_now = (WiFi.status() == WL_CONNECTED);
        State state = State::OfflineLocked;
        uint8_t reconnect_attempts = 0;
        uint32_t last_fetch_ms = 0;
        uint32_t last_reconnect_try_ms = 0;

        if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
        {
            state = state_;
            reconnect_attempts = reconnect_attempts_;
            last_fetch_ms = last_fetch_ms_;
            last_reconnect_try_ms = last_reconnect_try_ms_;
            xSemaphoreGive(mutex_);
        }

        switch (state)
        {
        case State::BootScan:
            setRuntimeState(State::Connecting, connected_now, false, 0);
            break;

        case State::Connecting:
            if (connectWithTimeout(cfg::wifi::kConnectTimeoutMs))
            {
                const bool weather_ok = fetchOpenMeteo();
                setRuntimeState(State::Online, true, weather_ok, 0);
            }
            else
            {
                WiFi.disconnect(false, false);
                setRuntimeState(State::OfflineLocked, false, false, 0);
            }
            break;

        case State::Online:
            if (!connected_now)
            {
                if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
                {
                    reconnect_attempts_ = 0;
                    last_reconnect_try_ms_ = 0;
                    xSemaphoreGive(mutex_);
                }
                setRuntimeState(State::ReconnectAttempts, false, false, 0);
                break;
            }

            if ((last_fetch_ms == 0U) || (now_ms - last_fetch_ms >= cfg::wifi::kWeatherRefreshMs))
            {
                const bool weather_ok = fetchOpenMeteo();
                setRuntimeState(State::Online, true, weather_ok, reconnect_attempts);
            }
            break;

        case State::ReconnectAttempts:
            if (connected_now)
            {
                const bool weather_ok = fetchOpenMeteo();
                setRuntimeState(State::Online, true, weather_ok, 0);
                break;
            }

            if ((last_reconnect_try_ms == 0U) || (now_ms - last_reconnect_try_ms >= cfg::wifi::kReconnectIntervalMs))
            {
                if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
                {
                    last_reconnect_try_ms_ = now_ms;
                    xSemaphoreGive(mutex_);
                }

                ++reconnect_attempts;
                const bool ok = connectWithTimeout(cfg::wifi::kConnectTimeoutMs);
                connected_now = ok;

                if (ok)
                {
                    const bool weather_ok = fetchOpenMeteo();
                    setRuntimeState(State::Online, true, weather_ok, 0);
                }
                else if (reconnect_attempts >= cfg::wifi::kReconnectMaxAttempts)
                {
                    WiFi.disconnect(false, false);
                    setRuntimeState(State::OfflineLocked, false, false, reconnect_attempts);
                }
                else
                {
                    setRuntimeState(State::ReconnectAttempts, false, false, reconnect_attempts);
                }
            }
            break;

        case State::OfflineLocked:
            setRuntimeState(State::OfflineLocked, false, false, reconnect_attempts);
            break;
        }

        vTaskDelay(wait_ticks);
    }
}
