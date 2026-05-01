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

        inline constexpr uint32_t kSensorTaskDelayMs = 200UL;
        inline constexpr uint32_t kSensorInitRetryMs = 1000UL;
        inline constexpr uint32_t kSensorLinkProbeMs = 5000UL;
        inline constexpr uint32_t kBootDataCheckMs = 20000UL;

        inline constexpr uint32_t kBsecStateSaveMs = 4UL * 60UL * 60UL * 1000UL; // 4 Hours
        inline constexpr uint32_t kBsecStateSaveMinGapMs = 10UL * 60UL * 1000UL; // 10 Minutes

        inline constexpr uint32_t kCpuLoadRefreshMs = 3000UL;
        inline constexpr uint32_t kSysInfoRefreshMs = 30000UL;
        // Page data refresh policy: do not update too frequently.
        inline constexpr uint32_t kUiValuesRefreshMs = 30000UL;
        inline constexpr uint32_t kUptimeRefreshMs = 1000UL;
    }

    namespace display
    {
        inline constexpr int kRotation = 0;
        inline constexpr int kWidth = 170;
        inline constexpr int kHeight = 320;

        inline constexpr int kCardWidth = 154;
        inline constexpr int kCardHeight = 60;
        inline constexpr int kCardGap = 8;
        inline constexpr int kCardStartY = 28;
        inline constexpr int kCardSecondRowY = 96;
        inline constexpr int kCardThirdRowY = 164;
        inline constexpr int kCardFourthRowY = 232;
        inline constexpr int kSysCardY = 35;
        inline constexpr int kSysCardHeight = 90;

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

        inline constexpr uint32_t kEstimatorSampleMs = 1000U;
        inline constexpr uint32_t kLabelRefreshMs = 10000U;

        inline constexpr uint32_t kLoadCompAwakeMv = 85U;
        inline constexpr uint32_t kLoadCompSleepMv = 25U;
        inline constexpr uint32_t kRestRecoveryBonusMv = 20U;
        inline constexpr uint32_t kRestDeltaMv = 7U;

        inline constexpr float kSocBlendActive = 0.08f;
        inline constexpr float kSocBlendSleep = 0.12f;
        inline constexpr float kSocBlendRest = 0.35f;
        inline constexpr float kSocBlendCharging = 0.22f;

        inline constexpr float kMaxDropPctPerMin = 1.8f;
        inline constexpr float kMaxRisePctPerMinIdle = 0.4f;
        inline constexpr float kMaxRisePctPerMinCharging = 7.0f;
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
        // Default offset is disabled because sensor placement is thermally isolated from the board.
        inline constexpr float kBsecTemperatureOffsetC = 0.0f;
        // Use this preset when a future compact design puts the sensor close to board heat.
        inline constexpr float kBsecTemperatureOffsetCompactC = 0.5f;
        inline constexpr float kKnownAltitudeMinM = -500.0f;
        inline constexpr float kKnownAltitudeMaxM = 9000.0f;

        inline constexpr float kGasTrendStableDeltaKohm = 0.35f;
        inline constexpr float kGasGaugeMinKohm = 0.0f;
        inline constexpr float kGasGaugeMaxKohm = 200.0f;
    }


    namespace color
    {
        inline constexpr uint32_t kBackground = 0x000000;
        inline constexpr uint32_t kCardBackground = 0x0A0A0A;
        inline constexpr uint32_t kBorder = 0x1E3A5F;
        inline constexpr uint32_t kTextDim = 0xA8B0C0;
        inline constexpr uint32_t kTextPrimary = 0xFFFFFF;
        inline constexpr uint32_t kValue = 0x00FFFF;
        inline constexpr uint32_t kStatusOk = 0x39FF14;
        inline constexpr uint32_t kError = 0xFF3B30;
        inline constexpr uint32_t kBootChecking = 0xFFD60A;

        inline constexpr uint32_t kBootGradientStart = 0x00FFFF;
        inline constexpr uint32_t kBootGradientEnd = 0x39FF14;
        inline constexpr uint32_t kBootAccent = 0xFF6B35;
        inline constexpr uint32_t kBootSuccess = 0x00FF88;
    }

    namespace boot_ui
    {
        inline constexpr uint32_t kTitleDelayMs = 150;
        inline constexpr uint32_t kLineDelayMs = 80;
        inline constexpr uint32_t kProgressBarMs = 200;
    }
} // namespace cfg