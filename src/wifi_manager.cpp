#include "wifi_manager.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <math.h>
#include <time.h>

#include "config.h"

WiFiManager &WiFiManager::instance()
{
    static WiFiManager manager;
    return manager;
}

int32_t WiFiManager::clampI32(int32_t value, int32_t low, int32_t high) const
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

void WiFiManager::ensureTimeSync()
{
    const uint32_t now_ms = millis();
    if (ntp_started_ && (last_ntp_sync_try_ms_ != 0U) && (now_ms - last_ntp_sync_try_ms_ < 10000UL))
    {
        return;
    }

    configTime(0, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
    ntp_started_ = true;
    last_ntp_sync_try_ms_ = now_ms;
}

void WiFiManager::setFetchError(FetchError error, int http_code)
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) != pdTRUE))
    {
        return;
    }

    last_fetch_error_ = error;
    last_http_status_code_ = http_code;
    xSemaphoreGive(mutex_);
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
        ensureTimeSync();
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
            ensureTimeSync();
            return true;
        }
        delay(120);
    }

    if (WiFi.status() == WL_CONNECTED)
    {
        ensureTimeSync();
    }
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiManager::parseNumberAfterKey(const String &src, const char *key, float &out_value) const
{
    int search_from = 0;
    while (search_from < src.length())
    {
        const int key_pos = src.indexOf(key, search_from);
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
        while ((colon < src.length()) &&
               ((src[colon] == ' ') || (src[colon] == '\t') || (src[colon] == '\r') || (src[colon] == '\n') ||
                (src[colon] == '"') || (src[colon] == '[')))
        {
            ++colon;
        }

        int end = colon;
        while ((end < src.length()) && ((src[end] == '-') || (src[end] == '+') || (src[end] == '.') || (src[end] >= '0' && src[end] <= '9')))
        {
            ++end;
        }

        if (end > colon)
        {
            out_value = src.substring(colon, end).toFloat();
            if (isfinite(out_value))
            {
                return true;
            }
        }

        search_from = key_pos + 1;
    }

    return false;
}

bool WiFiManager::parseWeatherPayload(const String &payload, WeatherSnapshot &snapshot) const
{
    const int cuaca_start = payload.indexOf("\"cuaca\"");
    if (cuaca_start < 0)
    {
        return false;
    }

    const int array_start = payload.indexOf("[", cuaca_start);
    const int array_end = payload.indexOf("]", array_start);
    if (array_start < 0 || array_end < 0)
    {
        return false;
    }

    String cuaca_block = payload.substring(array_start + 1, array_end);
    const int obj_start = cuaca_block.indexOf("{");
    const int obj_end = cuaca_block.indexOf("}");
    if (obj_start < 0 || obj_end < 0)
    {
        return false;
    }

    String first_obj = cuaca_block.substring(obj_start, obj_end + 1);

    float temp = NAN;
    parseNumberAfterKey(first_obj, "\"t\":", temp);
    if (!isfinite(temp))
    {
        parseNumberAfterKey(first_obj, "\"t\":", temp);
    }

    float humidity = NAN;
    parseNumberAfterKey(first_obj, "\"hu\":", humidity);

    float cloud_cov = NAN;
    parseNumberAfterKey(first_obj, "\"tcc\":", cloud_cov);
    uint8_t cloud_pct = 0;
    if (isfinite(cloud_cov))
    {
        cloud_pct = static_cast<uint8_t>(clampI32(static_cast<int32_t>(cloud_cov), 0, 100));
    }

    float precip = 0.0f;
    parseNumberAfterKey(first_obj, "\"tp\":", precip);

    uint8_t code = 0;
    float wcode = 0.0f;
    if (parseNumberAfterKey(first_obj, "\"weather\":", wcode))
    {
        code = static_cast<uint8_t>(clampI32(static_cast<int32_t>(wcode), 0, 99));
    }

    char weather_desc[32] = {0};
    const int desc_start = first_obj.indexOf("\"weather_desc\":\"");
    if (desc_start >= 0)
    {
        int desc_startq = desc_start + 16;
        int desc_endq = first_obj.indexOf("\"", desc_startq);
        if (desc_endq > desc_startq)
        {
            String desc = first_obj.substring(desc_startq, desc_endq);
            strncpy(weather_desc, desc.c_str(), sizeof(weather_desc) - 1);
        }
    }

    char local_dt[24] = {0};
    const int dt_start = first_obj.indexOf("\"local_datetime\":\"");
    if (dt_start >= 0)
    {
        int dt_startq = dt_start + 18;
        int dt_endq = first_obj.indexOf("\"", dt_startq);
        if (dt_endq > dt_startq)
        {
            String dt = first_obj.substring(dt_startq, dt_endq);
            strncpy(local_dt, dt.c_str(), sizeof(local_dt) - 1);

            int space_idx = dt.lastIndexOf(' ');
            if (space_idx > 0)
            {
                String time_str = dt.substring(space_idx + 1);
                if (time_str.length() >= 5)
                {
                    strncpy(snapshot.forecast_time, time_str.substring(0, 5).c_str(), sizeof(snapshot.forecast_time) - 1);
                }
            }
        }
    }

    if (!isfinite(temp))
    {
        return false;
    }

    snapshot.valid = true;
    snapshot.temperature_c = temp;
    snapshot.humidity_pct = humidity;
    snapshot.precipitation_mm = precip;
    snapshot.cloud_coverage_pct = cloud_pct;
    snapshot.weather_code = code;
    strncpy(snapshot.weather_desc, weather_desc, sizeof(snapshot.weather_desc) - 1);
    strncpy(snapshot.local_datetime, local_dt, sizeof(snapshot.local_datetime) - 1);
    snapshot.last_update_ms = millis();
    return true;
}

bool WiFiManager::fetchBmkg()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        setFetchError(FetchError::NotConnected);
        return false;
    }

    String url;
    url.reserve(384);
    url = "https://api.bmkg.go.id/publik/prakiraan-cuaca?adm4=32.73.14.1002";

    WiFiClientSecure secure_client;
    secure_client.setInsecure();

    HTTPClient http;
    http.setTimeout(static_cast<uint16_t>(cfg::wifi::kWeatherHttpTimeoutMs));
    if (!http.begin(secure_client, url))
    {
        setFetchError(FetchError::HttpBeginFailed);
        return false;
    }

    const int http_code = http.GET();
    if (http_code != HTTP_CODE_OK)
    {
        http.end();
        setFetchError(FetchError::HttpStatusFailed, http_code);
        return false;
    }

    const String payload = http.getString();
    http.end();
    if (payload.length() == 0)
    {
        setFetchError(FetchError::EmptyPayload);
        return false;
    }

    WeatherSnapshot snapshot{};
    if (!parseWeatherPayload(payload, snapshot))
    {
        setFetchError(FetchError::ParseFailed);
        return false;
    }

    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) != pdTRUE))
    {
        setFetchError(FetchError::MutexTimeout);
        return false;
    }

    snapshot_ = snapshot;
    const time_t now_epoch = time(nullptr);
    if (now_epoch > 1700000000)
    {
        snapshot_.last_update_epoch_utc = static_cast<uint32_t>(now_epoch);
    }
    else
    {
        snapshot_.last_update_epoch_utc = 0;
    }
    last_fetch_ms_ = snapshot.last_update_ms;
    last_fetch_error_ = FetchError::None;
    last_http_status_code_ = http_code;
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
        setFetchError(FetchError::NotConnected);
        return false;
    }

    const bool ok = fetchBmkg();
    setRuntimeState(ok ? State::Online : State::ReconnectAttempts,
                    WiFi.status() == WL_CONNECTED,
                    ok,
                    reconnect_attempts_);
    return ok;
}

const char *WiFiManager::lastErrorText() const
{
    if ((mutex_ == nullptr) || (xSemaphoreTake(mutex_, pdMS_TO_TICKS(30)) != pdTRUE))
    {
        return "state-lock-timeout";
    }

    const FetchError err = last_fetch_error_;
    xSemaphoreGive(mutex_);
    switch (err)
    {
    case FetchError::None:
        return "ok";
    case FetchError::NotConnected:
        return "wifi-not-connected";
    case FetchError::HttpBeginFailed:
        return "http-begin-failed";
    case FetchError::HttpStatusFailed:
        return "http-status-failed";
    case FetchError::EmptyPayload:
        return "empty-payload";
    case FetchError::ParseFailed:
        return "payload-parse-failed";
    case FetchError::PressureOutOfRange:
        return "pressure-out-of-range";
    case FetchError::MutexTimeout:
        return "state-mutex-timeout";
    }

    return "unknown";
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
    FetchError fetch_error = FetchError::None;
    int http_status = 0;
    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
    {
        state = state_;
        connected = connected_;
        online = online_mode_;
        attempts = reconnect_attempts_;
        fetch_error = last_fetch_error_;
        http_status = last_http_status_code_;
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
        out.printf("[WEATHER] temp=%.1fC humidity=%.0f%% clouds=%u%% desc=%s age=%lu ms\n",
                   snapshot.temperature_c,
                   snapshot.humidity_pct,
                   snapshot.cloud_coverage_pct,
                   snapshot.weather_desc[0] ? snapshot.weather_desc : "n/a",
                   static_cast<unsigned long>(age_ms));
    }
    else
    {
        out.printf("[WEATHER] no data (fetch not successful yet, reason=%s",
                   lastErrorText());
        if (fetch_error == FetchError::HttpStatusFailed)
        {
            out.printf(", http=%d", http_status);
        }
        out.println(")");
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
                const bool weather_ok = fetchBmkg();
                setRuntimeState(State::Online, true, weather_ok, 0);
            }
            else
            {
                if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
                {
                    reconnect_attempts_ = 0;
                    last_reconnect_try_ms_ = now_ms;
                    xSemaphoreGive(mutex_);
                }
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
                const bool weather_ok = fetchBmkg();
                setRuntimeState(State::Online, true, weather_ok, reconnect_attempts);
            }
            break;

        case State::ReconnectAttempts:
            if (connected_now)
            {
                const bool weather_ok = fetchBmkg();
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
                    const bool weather_ok = fetchBmkg();
                    setRuntimeState(State::Online, true, weather_ok, 0);
                }
                else if (reconnect_attempts >= cfg::wifi::kReconnectMaxAttempts)
                {
                    if ((mutex_ != nullptr) && (xSemaphoreTake(mutex_, pdMS_TO_TICKS(60)) == pdTRUE))
                    {
                        reconnect_attempts_ = reconnect_attempts;
                        last_reconnect_try_ms_ = now_ms;
                        xSemaphoreGive(mutex_);
                    }
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
            if ((reconnect_attempts < cfg::wifi::kReconnectMaxAttempts) &&
                ((last_reconnect_try_ms == 0U) || (now_ms - last_reconnect_try_ms >= cfg::wifi::kReconnectIntervalMs)))
            {
                setRuntimeState(State::ReconnectAttempts, false, false, reconnect_attempts);
            }
            break;
        }

        vTaskDelay(wait_ticks);
    }
}