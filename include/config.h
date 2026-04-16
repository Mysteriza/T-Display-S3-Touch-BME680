#pragma once

#include <Arduino.h>
#include <cstdint>
namespace cfg
{
    namespace pins
    {
        inline constexpr int kLcdBacklight = 38;
        inline constexpr int kButtonWake = 14;

        inline constexpr int kLcdPower = 15;
        inline constexpr int kLcdCs = 6;
        inline constexpr int kLcdDc = 7;
        inline constexpr int kLcdRst = 5;
        inline constexpr int kLcdWr = 8;
        inline constexpr int kLcdRd = 9;

        inline constexpr int kLcdD0 = 39;
        inline constexpr int kLcdD1 = 40;
        inline constexpr int kLcdD2 = 41;
        inline constexpr int kLcdD3 = 42;
        inline constexpr int kLcdD4 = 45;
        inline constexpr int kLcdD5 = 46;
        inline constexpr int kLcdD6 = 47;
        inline constexpr int kLcdD7 = 48;

        inline constexpr int kTouchRst = 21;
        inline constexpr int kTouchInt = 16;
        inline constexpr uint8_t kTouchAddrPrimary = 0x15;
        inline constexpr uint8_t kTouchAddrSecondary = 0x1A;

        inline constexpr int kBatteryAdc = 4;

        inline constexpr int kMainI2cSda = 18;
        inline constexpr int kMainI2cScl = 17;
        inline constexpr int kAltI2cSda = 43;
        inline constexpr int kAltI2cScl = 44;
        inline constexpr uint8_t kBmeAddressConfigured = 0x76;
    }

    namespace timing
    {
        inline constexpr uint32_t kDisplayTimeoutMs = 15000UL;
        inline constexpr uint32_t kButtonDebounceMs = 120UL;

        inline constexpr uint32_t kSensorRefreshMs = 30000UL;
        inline constexpr uint32_t kSensorStaleReinitMs = kSensorRefreshMs + 5000UL;

        inline constexpr uint32_t kUiTaskActiveDelayMs = 16UL;
        inline constexpr uint32_t kUiTaskSleepDelayMs = 120UL;
        inline constexpr uint32_t kUiBackgroundRefreshMs = kSensorRefreshMs;

        inline constexpr uint32_t kSensorTaskDelayMs = 200UL;
        inline constexpr uint32_t kSensorInitRetryMs = 1000UL;
        inline constexpr uint32_t kSensorLinkProbeMs = 5000UL;
        inline constexpr uint32_t kBootDataCheckMs = 5000UL;

        inline constexpr uint32_t kBsecStateSaveMs = 4UL * 60UL * 60UL * 1000UL; // 4 Hours
        inline constexpr uint32_t kBsecStateSaveMinGapMs = 10UL * 60UL * 1000UL; // 10 Minutes
        inline constexpr uint32_t kIaqStuckTimeoutMs = 20UL * 60UL * 1000UL;

        inline constexpr uint32_t kCpuLoadRefreshMs = kSensorRefreshMs;
        inline constexpr uint32_t kSysInfoRefreshMs = kSensorRefreshMs;
        inline constexpr uint32_t kUiValuesRefreshMs = kSensorRefreshMs;
        inline constexpr uint32_t kUptimeRefreshMs = 1000UL;
    }

    namespace display
    {
        inline constexpr int kRotation = 3;
        inline constexpr int kWidth = 320;
        inline constexpr int kHeight = 170;

        inline constexpr int kCardWidth = 148;
        inline constexpr int kCardHeight = 58;
        inline constexpr int kCardGap = 8;
        inline constexpr int kCardStartY = 30;
        inline constexpr int kCardSecondRowY = 94;

        inline constexpr int kMarginLeft = 8;
        inline constexpr int kMarginRight = 8;
        inline constexpr int kHeaderY = 7;
        inline constexpr int kHeaderRightY = 8;
    }

    namespace power
    {
        inline constexpr uint32_t kCpuFreqActiveMhz = 240U;
        inline constexpr uint32_t kCpuFreqSleepMhz = 80U;
    }

    namespace battery
    {
        inline constexpr uint8_t kAdcSamples = 8;
        inline constexpr uint32_t kMinMv = 3200U;
        inline constexpr uint32_t kMaxMv = 4200U;
        inline constexpr uint32_t kAbsentMv = 4300U;
        inline constexpr uint8_t kDownConfirmCycles = 2;
        inline constexpr uint8_t kUpConfirmCycles = 4;
        inline constexpr uint8_t kUpMinStep = 2;
    }

    namespace sensor
    {
        inline constexpr uint8_t kRunFailLimit = 2;
        inline constexpr uint8_t kLinkOkLimit = 2;
        inline constexpr uint8_t kLinkFailLimit = 2;
        inline constexpr uint8_t kBmeChipIdReg = 0xD0;
        inline constexpr uint8_t kBmeChipIdValue = 0x61;

        inline constexpr float kSeaLevelDefaultHpa = 1013.25f;
        inline constexpr float kSeaLevelMinHpa = 850.0f;
        inline constexpr float kSeaLevelMaxHpa = 1100.0f;
        inline constexpr float kKnownAltitudeMinM = -500.0f;
        inline constexpr float kKnownAltitudeMaxM = 9000.0f;
        inline constexpr float kIaqStuckTarget = 50.0f;
        inline constexpr float kIaqStuckTolerance = 2.0f;
    }

    namespace color
    {
        inline constexpr uint32_t kBackground = 0x000000;
        inline constexpr uint32_t kCardBackground = 0x0A0A0A;
        inline constexpr uint32_t kBorder = 0x1E3A5F;
        inline constexpr uint32_t kTextDim = 0xA8B0C0;
        inline constexpr uint32_t kValue = 0x00FFFF;
        inline constexpr uint32_t kStatusOk = 0x39FF14;
        inline constexpr uint32_t kError = 0xFF3B30;
        inline constexpr uint32_t kBootChecking = 0xFFD60A;
    }
} // namespace cfg