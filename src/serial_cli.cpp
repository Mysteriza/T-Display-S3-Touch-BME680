#include "serial_cli.h"

#include "sensor_manager.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "wifi_manager.h"

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

    if ((argc >= 2U) && (strcmp(argv[0], "wifi") == 0) && (strcmp(argv[1], "status") == 0))
    {
        WiFiManager::instance().printStatus(Serial);
        return;
    }

    if ((argc >= 2U) && (strcmp(argv[0], "weather") == 0) && (strcmp(argv[1], "status") == 0))
    {
        WiFiManager::instance().printStatus(Serial);
        return;
    }

    if ((argc >= 3U) && (strcmp(argv[0], "weather") == 0) && (strcmp(argv[1], "fetch") == 0) && (strcmp(argv[2], "now") == 0))
    {
        if (WiFiManager::instance().forceFetchNow())
        {
            Serial.println("[CMD] weather fetch succeeded.");
        }
        else
        {
            Serial.println("[CMD] weather fetch failed (offline or internet unavailable).");
        }
        return;
    }

    Serial.printf("[CMD] unknown command: %s\n", original);
    Serial.println("[CMD] type 'help' for available commands.");
}

void SerialCLI::tick()
{
    bool saw_input = false;
    while (Serial.available() > 0)
    {
        const int raw = Serial.read();
        if (raw < 0)
        {
            break;
        }
        saw_input = true;

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
            last_input_ms_ = millis();
        }
        else
        {
            line_len_ = 0;
            line_buffer_[0] = '\0';
            Serial.println("[CMD] input too long, line dropped.");
        }
    }

    if (!saw_input && (line_len_ > 0U) && (last_input_ms_ != 0U))
    {
        const uint32_t now_ms = millis();
        if (now_ms - last_input_ms_ >= 250U)
        {
            line_buffer_[line_len_] = '\0';
            handleCommand(line_buffer_);
            line_len_ = 0;
            line_buffer_[0] = '\0';
            last_input_ms_ = 0;
        }
    }
}
