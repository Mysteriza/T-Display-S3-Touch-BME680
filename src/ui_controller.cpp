#include "ui_controller.h"

#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>

#include "config.h"
#include "i2c_bus_lock.h"
#include "power_mgmt.h"
#include "sensor_manager.h"
#include "wifi_manager.h"

extern const lv_font_t lv_font_montserrat_18;
extern const lv_font_t lv_font_montserrat_20;
extern const lv_font_t lv_font_montserrat_22;
extern const lv_font_t lv_font_montserrat_14;
extern const lv_font_t lv_font_montserrat_32;
extern const lv_font_t lv_font_montserrat_36;
extern const lv_font_t lv_font_montserrat_48;
extern const lv_font_t lv_font_montserrat_12;

#if !defined(LV_FONT_MONTSERRAT_18) || (LV_FONT_MONTSERRAT_18 == 0)
#define lv_font_montserrat_18 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_20) || (LV_FONT_MONTSERRAT_20 == 0)
#define lv_font_montserrat_20 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_22) || (LV_FONT_MONTSERRAT_22 == 0)
#define lv_font_montserrat_22 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_14) || (LV_FONT_MONTSERRAT_14 == 0)
#define lv_font_montserrat_14 LV_FONT_DEFAULT
#endif
#if !defined(LV_FONT_MONTSERRAT_32) || (LV_FONT_MONTSERRAT_32 == 0)
#define lv_font_montserrat_32 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_36) || (LV_FONT_MONTSERRAT_36 == 0)
#define lv_font_montserrat_36 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_48) || (LV_FONT_MONTSERRAT_48 == 0)
#define lv_font_montserrat_48 lv_font_montserrat_14
#endif
#if !defined(LV_FONT_MONTSERRAT_12) || (LV_FONT_MONTSERRAT_12 == 0)
#define lv_font_montserrat_12 lv_font_montserrat_14
#endif

UiController &UiController::instance()
{
    static UiController controller;
    return controller;
}

int32_t UiController::clampI32(int32_t value, int32_t low, int32_t high) const
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

int32_t UiController::roundToInt(float value) const
{
    if (!isfinite(value))
    {
        return 0;
    }
    return static_cast<int32_t>(lroundf(value));
}

void UiController::formatUptime(char *out, size_t out_len, uint64_t uptime_seconds) const
{
    const uint64_t days = uptime_seconds / 86400ULL;
    const uint64_t hours = (uptime_seconds % 86400ULL) / 3600ULL;
    const uint64_t minutes = (uptime_seconds % 3600ULL) / 60ULL;
    const uint64_t seconds = uptime_seconds % 60ULL;

    if (days > 0ULL)
    {
        snprintf(out, out_len, "%llud %02llu:%02llu:%02llu",
                 static_cast<unsigned long long>(days),
                 static_cast<unsigned long long>(hours),
                 static_cast<unsigned long long>(minutes),
                 static_cast<unsigned long long>(seconds));
        return;
    }

    snprintf(out, out_len, "%02llu:%02llu:%02llu",
             static_cast<unsigned long long>(hours),
             static_cast<unsigned long long>(minutes),
             static_cast<unsigned long long>(seconds));
}

uint8_t UiController::batteryPercentFromMv(uint32_t mv) const
{
    static const uint16_t kOcvMv[] = {
        3200,
        3300,
        3400,
        3500,
        3600,
        3700,
        3750,
        3800,
        3850,
        3900,
        3950,
        4000,
        4050,
        4100,
        4150,
        4200,
    };
    static const uint8_t kSocPct[] = {
        0,
        1,
        3,
        6,
        11,
        22,
        30,
        38,
        47,
        56,
        65,
        74,
        82,
        90,
        96,
        100,
    };

    if (mv <= kOcvMv[0])
    {
        return 0;
    }

    const size_t count = sizeof(kOcvMv) / sizeof(kOcvMv[0]);
    if (mv >= kOcvMv[count - 1U])
    {
        return 100;
    }

    for (size_t i = 0; i + 1U < count; ++i)
    {
        const uint16_t low_mv = kOcvMv[i];
        const uint16_t high_mv = kOcvMv[i + 1U];
        if ((mv < low_mv) || (mv > high_mv))
        {
            continue;
        }

        const uint8_t low_soc = kSocPct[i];
        const uint8_t high_soc = kSocPct[i + 1U];
        const uint32_t span_mv = static_cast<uint32_t>(high_mv - low_mv);
        const uint32_t progress_mv = static_cast<uint32_t>(mv - low_mv);
        const uint32_t interpolated = static_cast<uint32_t>(low_soc) +
                                      ((static_cast<uint32_t>(high_soc - low_soc) * progress_mv + (span_mv / 2U)) / span_mv);
        return static_cast<uint8_t>(interpolated);
    }

    return 0;
}

uint32_t UiController::estimateBatteryOcvMv(uint32_t measured_mv, bool display_awake) const
{
    const uint32_t load_comp_base = display_awake ? cfg::battery::kLoadCompAwakeMv : cfg::battery::kLoadCompSleepMv;
    const float cpu_factor = static_cast<float>(clampI32(static_cast<int32_t>(lroundf(cpu_load_estimate_pct_)), 0, 100)) / 100.0f;
    const uint32_t load_comp = static_cast<uint32_t>(lroundf(static_cast<float>(load_comp_base) * (0.30f + (0.70f * cpu_factor))));

    uint32_t ocv_mv = measured_mv + load_comp;

    const uint32_t ripple_mv = (bat_fast_mv_ >= bat_slow_mv_) ? (bat_fast_mv_ - bat_slow_mv_) : (bat_slow_mv_ - bat_fast_mv_);
    if ((!display_awake) && (ripple_mv <= cfg::battery::kRestDeltaMv))
    {
        ocv_mv += cfg::battery::kRestRecoveryBonusMv;
    }

    if (ocv_mv < cfg::battery::kMinMv)
    {
        ocv_mv = cfg::battery::kMinMv;
    }
    if (ocv_mv > cfg::battery::kMaxMv)
    {
        ocv_mv = cfg::battery::kMaxMv;
    }

    return ocv_mv;
}

uint8_t UiController::estimateUiTaskLoadPercent() const
{
    const int32_t rounded = clampI32(static_cast<int32_t>(lroundf(cpu_load_estimate_pct_)), 0, 100);
    return static_cast<uint8_t>(rounded);
}

lv_color_t UiController::blendHexColors(uint32_t from_hex, uint32_t to_hex, uint8_t mix_255) const
{
    const uint32_t inv = 255U - mix_255;

    const uint32_t from_r = (from_hex >> 16) & 0xFFU;
    const uint32_t from_g = (from_hex >> 8) & 0xFFU;
    const uint32_t from_b = from_hex & 0xFFU;

    const uint32_t to_r = (to_hex >> 16) & 0xFFU;
    const uint32_t to_g = (to_hex >> 8) & 0xFFU;
    const uint32_t to_b = to_hex & 0xFFU;

    const uint32_t r = (from_r * inv + to_r * mix_255 + 127U) / 255U;
    const uint32_t g = (from_g * inv + to_g * mix_255 + 127U) / 255U;
    const uint32_t b = (from_b * inv + to_b * mix_255 + 127U) / 255U;

    return lv_color_make(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b));
}

lv_color_t UiController::batteryColorFromPercent(uint8_t percent, bool usb_power) const
{
    if (usb_power)
    {
        return lv_color_hex(0x2EDC72);
    }

    static const uint8_t stops_percent[] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10};
    static const uint32_t stops_color[] = {
        0x2EDC72,
        0x4FD968,
        0x73D55D,
        0x97D153,
        0xBCD049,
        0xD9C444,
        0xEEAF3E,
        0xF99339,
        0xFD6D35,
        cfg::color::kError,
    };

    const size_t stop_count = sizeof(stops_percent) / sizeof(stops_percent[0]);

    const uint8_t p = static_cast<uint8_t>(clampI32(percent, 10, 100));
    for (size_t i = 0; i + 1U < stop_count; ++i)
    {
        const uint8_t high = stops_percent[i];
        const uint8_t low = stops_percent[i + 1U];
        if ((p <= high) && (p >= low))
        {
            const uint8_t span = static_cast<uint8_t>(high - low);
            const uint8_t progress = static_cast<uint8_t>(high - p);
            const uint8_t mix = (span == 0U) ? 0U : static_cast<uint8_t>((255U * progress) / span);
            return blendHexColors(stops_color[i], stops_color[i + 1U], mix);
        }
    }

    return lv_color_hex(stops_color[stop_count - 1U]);
}

namespace
{
    uint32_t tempValueColor(float c)
    {
        if (!isfinite(c))
        {
            return cfg::color::kTextDim;
        }
        if (c < 16.0f)
        {
            return cfg::color::kBootChecking;
        }
        if (c <= 27.0f)
        {
            return cfg::color::kStatusOk;
        }
        if (c <= 32.0f)
        {
            return cfg::color::kBootChecking;
        }
        return cfg::color::kError;
    }

    uint32_t humidityValueColor(float h)
    {
        if (!isfinite(h))
        {
            return cfg::color::kTextDim;
        }
        if ((h >= 40.0f) && (h <= 60.0f))
        {
            return cfg::color::kStatusOk;
        }
        if (((h >= 30.0f) && (h < 40.0f)) || ((h > 60.0f) && (h <= 70.0f)))
        {
            return cfg::color::kBootChecking;
        }
        return cfg::color::kError;
    }

    uint32_t gasValueColor(float gas_kohm)
    {
        if (!isfinite(gas_kohm))
        {
            return cfg::color::kTextDim;
        }
        if (gas_kohm <= 50.0f)
        {
            return cfg::color::kStatusOk;
        }
        if (gas_kohm <= 100.0f)
        {
            return cfg::color::kBootChecking;
        }
        if (gas_kohm <= 150.0f)
        {
            return 0xFF8C32;
        }
        return cfg::color::kError;
    }

    const char *gasStatusText(float gas_kohm)
    {
        if (!isfinite(gas_kohm))
        {
            return "No Data";
        }
        if (gas_kohm <= 50.0f)
        {
            return "Good";
        }
        if (gas_kohm <= 100.0f)
        {
            return "Moderate";
        }
        if (gas_kohm <= 150.0f)
        {
            return "Unhealthy";
        }
        return "Hazardous";
    }

    const char *gasTrendText(int8_t trend)
    {
        if (trend > 0)
        {
            return "Rising";
        }
        if (trend < 0)
        {
            return "Falling";
        }
        return "Stable";
    }

    uint32_t gasTrendColor(int8_t trend)
    {
        if (trend > 0)
        {
            return cfg::color::kError;
        }
        if (trend < 0)
        {
            return cfg::color::kBootChecking;
        }
        return cfg::color::kStatusOk;
    }

    const char *weatherConditionText(uint8_t code)
    {
        switch (code)
        {
        case 0:
            return "Clear";
        case 1:
            return "Mainly Clear";
        case 2:
            return "Partly Cloudy";
        case 3:
            return "Overcast";
        case 45:
        case 48:
            return "Fog";
        case 51:
        case 53:
        case 55:
            return "Drizzle";
        case 56:
        case 57:
            return "Freezing Drizzle";
        case 61:
        case 63:
        case 65:
            return "Rain";
        case 66:
        case 67:
            return "Freezing Rain";
        case 71:
        case 73:
        case 75:
            return "Snow";
        case 77:
            return "Snow Grains";
        case 80:
        case 81:
        case 82:
            return "Rain Showers";
        case 85:
        case 86:
            return "Snow Showers";
        case 95:
            return "Thunderstorm";
        case 96:
        case 99:
            return "Thunder + Hail";
        default:
            return "Unknown";
        }
    }

    uint32_t weatherConditionColor(uint8_t code)
    {
        if (code == 0 || code == 1)
        {
            return cfg::color::kStatusOk;
        }
        if (code == 2 || code == 3)
        {
            return cfg::color::kBootChecking;
        }
        if (code >= 45 && code <= 57)
        {
            return 0x5DADE2;
        }
        if (code >= 61 && code <= 67)
        {
            return 0x3498DB;
        }
        if (code >= 71 && code <= 77)
        {
            return 0xECF0F1;
        }
        if (code >= 80 && code <= 82)
        {
            return 0x2980B9;
        }
        if (code >= 85 && code <= 86)
        {
            return 0xBDC3C7;
        }
        if (code >= 95)
        {
            return cfg::color::kError;
        }
        return cfg::color::kTextDim;
    }

    uint32_t rainAmountColor(float precipitation_mm)
    {
        if (!isfinite(precipitation_mm) || precipitation_mm < 0.0f)
        {
            return cfg::color::kTextDim;
        }
        if (precipitation_mm <= 0.0f)
        {
            return cfg::color::kStatusOk;
        }
        if (precipitation_mm < 1.0f)
        {
            return cfg::color::kBootChecking;
        }
        if (precipitation_mm < 5.0f)
        {
            return 0xFF8C32;
        }
        return cfg::color::kError;
    }
}

bool UiController::probeTouchAddress(uint8_t address)
{
    ScopedI2cBusLock lock(pdMS_TO_TICKS(40));
    if (!lock.ownsLock())
    {
        return false;
    }

    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

void UiController::syncTouchPowerState(bool display_awake)
{
    if (!touch_ready_)
    {
        return;
    }

    if (display_awake)
    {
        if (!touch_hw_awake_)
        {
            touch_.wakeup();
            delay(3);
            touch_hw_awake_ = true;
        }
    }
    else
    {
        if (touch_hw_awake_)
        {
            touch_.sleep();
            touch_hw_awake_ = false;
        }
    }
}

bool UiController::initDisplay()
{
    pinMode(cfg::pins::kLcdPower, OUTPUT);
    digitalWrite(cfg::pins::kLcdPower, HIGH);
    pinMode(cfg::pins::kLcdBacklight, OUTPUT);
    digitalWrite(cfg::pins::kLcdBacklight, HIGH);
    pinMode(cfg::pins::kBatteryAdc, INPUT);

    lv_init();

    tft_.begin();
    tft_.setRotation(cfg::display::kRotation);
    tft_.fillScreen(TFT_BLACK);

    const size_t buf_pixels = static_cast<size_t>(cfg::display::kWidth) * 20U;
    buf_a_ = static_cast<lv_color_t *>(heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    buf_b_ = static_cast<lv_color_t *>(heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    if ((buf_a_ == nullptr) || (buf_b_ == nullptr))
    {
        return false;
    }

    lv_disp_draw_buf_init(&draw_buf_, buf_a_, buf_b_, buf_pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = cfg::display::kWidth;
    disp_drv.ver_res = cfg::display::kHeight;
    disp_drv.flush_cb = UiController::displayFlushCallback;
    disp_drv.draw_buf = &draw_buf_;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = UiController::touchReadCallback;
    touch_indev_ = lv_indev_drv_register(&indev_drv);

    lvgl_ready_ = true;
    return true;
}

bool UiController::initTouch()
{
    pinMode(cfg::pins::kTouchRst, OUTPUT);
    pinMode(cfg::pins::kTouchInt, INPUT_PULLUP);
    {
        ScopedI2cBusLock lock(pdMS_TO_TICKS(200));
        if (!lock.ownsLock())
        {
            touch_ready_ = false;
            return false;
        }
        Wire.begin();
        Wire.setClock(400000);
    }

    digitalWrite(cfg::pins::kTouchRst, LOW);
    delay(30);
    digitalWrite(cfg::pins::kTouchRst, HIGH);
    delay(80);

    touch_addr_ = 0;
    if (probeTouchAddress(cfg::pins::kTouchAddrPrimary))
    {
        touch_addr_ = cfg::pins::kTouchAddrPrimary;
    }
    else if (probeTouchAddress(cfg::pins::kTouchAddrSecondary))
    {
        touch_addr_ = cfg::pins::kTouchAddrSecondary;
    }

    if (touch_addr_ == 0)
    {
        touch_ready_ = false;
        return false;
    }

    touch_.setPins(cfg::pins::kTouchRst, cfg::pins::kTouchInt);
    {
        ScopedI2cBusLock lock(pdMS_TO_TICKS(200));
        touch_ready_ = lock.ownsLock() && touch_.begin(Wire, touch_addr_, -1, -1);
    }
    if (!touch_ready_)
    {
        return false;
    }

    touch_.setMaxCoordinates(cfg::display::kWidth, cfg::display::kHeight);
    touch_.setMirrorXY(true, false);
    touch_.setSwapXY(true);
    touch_.disableAutoSleep();
    touch_.setCenterButtonCoordinate(85, 360);

    touch_hw_awake_ = true;
    prev_display_awake_ = PowerManager::instance().isDisplayAwake();
    syncTouchPowerState(prev_display_awake_);

    PowerManager::instance().wakeFromInteraction();
    return true;
}

void UiController::setBootLine(lv_obj_t *label, const char *name, bool done, bool ok)
{
    if (label == nullptr)
    {
        return;
    }

    bool wifi_line = false;
    if (name != nullptr)
    {
        wifi_line = (strcmp(name, "WiFi Boot") == 0);
    }

    uint32_t status_color = cfg::color::kBootChecking;
    const char *status_text = "[...]";
    if (done)
    {
        if (ok)
        {
            status_color = cfg::color::kStatusOk;
            status_text = "[OK]";
        }
        else if (wifi_line)
        {
            status_color = cfg::color::kBootChecking;
            status_text = "[OFFLINE]";
        }
        else
        {
            status_color = cfg::color::kError;
            status_text = "[FAIL]";
        }
    }

    char line[96] = {0};
    snprintf(line, sizeof(line), "%-12s %s", name, status_text);
    lv_label_set_recolor(label, false);
    lv_obj_set_style_text_color(label, lv_color_hex(status_color), 0);
    lv_label_set_text(label, line);
}

void UiController::bootDiagBegin()
{
    if (!lvgl_ready_)
    {
        return;
    }

    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen, lv_color_hex(cfg::color::kBackground), 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    boot_title_ = lv_label_create(screen);
    lv_label_set_text(boot_title_, "ENV_MONITOR");
    lv_obj_set_style_text_color(boot_title_, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(boot_title_, &lv_font_montserrat_22, 0);
    lv_obj_align(boot_title_, LV_ALIGN_TOP_MID, 0, 8);

    boot_subtitle_ = lv_label_create(screen);
    lv_label_set_text(boot_subtitle_, "System Self-Check");
    lv_obj_set_style_text_color(boot_subtitle_, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(boot_subtitle_, &lv_font_montserrat_12, 0);
    lv_obj_align(boot_subtitle_, LV_ALIGN_TOP_MID, 0, 28);

    const lv_coord_t line_x = 24;
    const lv_coord_t y0 = 50;
    const lv_coord_t step = 18;

    boot_items_[0].label = lv_label_create(screen);
    boot_items_[0].icon = lv_label_create(screen);
    boot_items_[0].name = "LCD";
    boot_items_[0].done = false;
    boot_items_[0].ok = false;

    boot_items_[1].label = lv_label_create(screen);
    boot_items_[1].icon = lv_label_create(screen);
    boot_items_[1].name = "Touch";
    boot_items_[1].done = false;
    boot_items_[1].ok = false;

    boot_items_[2].label = lv_label_create(screen);
    boot_items_[2].icon = lv_label_create(screen);
    boot_items_[2].name = "Sensor";
    boot_items_[2].done = false;
    boot_items_[2].ok = false;

    boot_items_[3].label = lv_label_create(screen);
    boot_items_[3].icon = lv_label_create(screen);
    boot_items_[3].name = "Gas Data";
    boot_items_[3].done = false;
    boot_items_[3].ok = false;

    boot_items_[4].label = lv_label_create(screen);
    boot_items_[4].icon = lv_label_create(screen);
    boot_items_[4].name = "WiFi";
    boot_items_[4].done = false;
    boot_items_[4].ok = false;

    for (uint8_t i = 0; i < 5U; ++i)
    {
        lv_obj_set_style_text_font(boot_items_[i].label, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_font(boot_items_[i].icon, &lv_font_montserrat_12, 0);
        lv_obj_set_style_text_color(boot_items_[i].label, lv_color_hex(cfg::color::kTextDim), 0);
        lv_obj_set_style_text_color(boot_items_[i].icon, lv_color_hex(cfg::color::kTextDim), 0);
        lv_label_set_text(boot_items_[i].icon, " ");
        lv_obj_align(boot_items_[i].icon, LV_ALIGN_TOP_LEFT, line_x - 12, y0 + (step * i));
        char line[48] = {0};
        snprintf(line, sizeof(line), "%-8s [..]", boot_items_[i].name);
        lv_label_set_text(boot_items_[i].label, line);
        lv_obj_align(boot_items_[i].label, LV_ALIGN_TOP_LEFT, line_x, y0 + (step * i));
    }

    boot_version_ = lv_label_create(screen);
    lv_label_set_text(boot_version_, "v0.2.0");
    lv_obj_set_style_text_color(boot_version_, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(boot_version_, &lv_font_montserrat_12, 0);
    lv_obj_align(boot_version_, LV_ALIGN_BOTTOM_MID, 0, -4);

    boot_current_item_ = 0;
    boot_animating_ = false;
    boot_animation_step_ = 0;
    boot_last_anim_ms_ = 0;

    lv_scr_load(screen);
}

void UiController::bootDiagUpdate(const BootDiagStatus &status)
{
    if (!lvgl_ready_)
    {
        return;
    }

    for (uint8_t i = 0; i < 5U; ++i)
    {
        const BootItem &item = boot_items_[i];
        if (item.label == nullptr)
        {
            continue;
        }

        uint32_t color = cfg::color::kTextDim;
        const char *status_str = "[..]";
        bool show = false;

        if (i == 0)
        {
            show = status.lcd_done;
            if (show)
            {
                color = status.lcd_ok ? cfg::color::kStatusOk : cfg::color::kError;
                status_str = status.lcd_ok ? "[OK]" : "[FAIL]";
            }
        }
        else if (i == 1)
        {
            show = status.touch_done;
            if (show)
            {
                color = status.touch_ok ? cfg::color::kStatusOk : cfg::color::kError;
                status_str = status.touch_ok ? "[OK]" : "[FAIL]";
            }
        }
        else if (i == 2)
        {
            show = status.sensor_done;
            if (show)
            {
                color = status.sensor_ok ? cfg::color::kStatusOk : cfg::color::kError;
                status_str = status.sensor_ok ? "[OK]" : "[FAIL]";
            }
        }
        else if (i == 3)
        {
            show = status.data_done;
            if (show)
            {
                color = status.data_ok ? cfg::color::kStatusOk : cfg::color::kError;
                status_str = status.data_ok ? "[OK]" : "[FAIL]";
            }
        }
        else if (i == 4)
        {
            show = status.wifi_done;
            if (show)
            {
                color = status.wifi_ok ? cfg::color::kStatusOk : cfg::color::kBootChecking;
                status_str = status.wifi_ok ? "[OK]" : "[OFFLINE]";
            }
        }

        if (show)
        {
            char line[48] = {0};
            snprintf(line, sizeof(line), "%-8s %s", item.name, status_str);
            lv_label_set_text(item.label, line);
            lv_obj_set_style_text_color(item.label, lv_color_hex(color), 0);
        }
    }

    lv_tick_inc(20);
    lv_timer_handler();
    delay(20);
}

void UiController::bootDiagFinish(uint32_t hold_ms)
{
    if (!lvgl_ready_)
    {
        return;
    }

    const uint32_t start = millis();
    while (millis() - start < hold_ms)
    {
        lv_tick_inc(20);
        lv_timer_handler();
        delay(20);
    }
}

lv_obj_t *UiController::createHeader(lv_obj_t *parent, const char *page_info, uint8_t page_index)
{
    lv_obj_t *left = lv_label_create(parent);
    lv_label_set_text(left, LV_SYMBOL_DIRECTORY " Env");
    lv_obj_set_style_text_color(left, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(left, &lv_font_montserrat_14, 0);
    lv_obj_align(left, LV_ALIGN_TOP_LEFT, cfg::display::kMarginLeft, cfg::display::kHeaderY);

    lv_obj_t *right = lv_label_create(parent);
    lv_label_set_text(right, page_info);
    lv_obj_set_style_text_color(right, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(right, &lv_font_montserrat_12, 0);
    lv_obj_align(right, LV_ALIGN_TOP_RIGHT, -cfg::display::kMarginRight, cfg::display::kHeaderRightY);

    if (page_index < 4U)
    {
        env_headers_[page_index] = left;
    }

    return left;
}

lv_obj_t *UiController::createValueCard(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                                        const char *title, lv_obj_t **value_label, const lv_font_t *value_font,
                                        const lv_font_t *title_font)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_bg_color(card, lv_color_hex(cfg::color::kCardBackground), 0);
    lv_obj_set_style_border_color(card, lv_color_hex(cfg::color::kBorder), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 8, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *title_label = lv_label_create(card);
    lv_label_set_text(title_label, title);
    lv_obj_set_style_text_color(title_label, lv_color_hex(cfg::color::kTextDim), 0);
    if (title_font != nullptr)
    {
        lv_obj_set_style_text_font(title_label, title_font, 0);
    }
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

    *value_label = lv_label_create(card);
    lv_label_set_text(*value_label, "--");
    lv_obj_set_style_text_color(*value_label, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(*value_label, value_font, 0);
    lv_obj_align(*value_label, LV_ALIGN_BOTTOM_LEFT, 0, 2);

    return card;
}

lv_obj_t *UiController::createSingleLineCard(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                                             const char *text, lv_obj_t **text_label, const lv_font_t *text_font)
{
    lv_obj_t *card = lv_obj_create(parent);
    lv_obj_set_size(card, w, h);
    lv_obj_set_pos(card, x, y);
    lv_obj_set_style_bg_color(card, lv_color_hex(cfg::color::kCardBackground), 0);
    lv_obj_set_style_border_color(card, lv_color_hex(cfg::color::kBorder), 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_left(card, 8, 0);
    lv_obj_set_style_pad_right(card, 8, 0);
    lv_obj_set_style_pad_top(card, 6, 0);
    lv_obj_set_style_pad_bottom(card, 6, 0);
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

    *text_label = lv_label_create(card);
    lv_label_set_text(*text_label, text);
    lv_obj_set_style_text_color(*text_label, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(*text_label, text_font, 0);
    lv_obj_align(*text_label, LV_ALIGN_LEFT_MID, 0, 0);

    return card;
}

void UiController::buildPageEnv(lv_obj_t *parent)
{
    createHeader(parent, "01 / ENV", 0);

    battery_labels_[0] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[0], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[0], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[0], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[0], LV_ALIGN_TOP_MID, 0, 8);

    const lv_coord_t x0 = cfg::display::kMarginLeft;
    const lv_coord_t x1 = x0 + cfg::display::kCardWidth + cfg::display::kCardGap;

    createValueCard(parent,
                    x0,
                    cfg::display::kCardStartY,
                    cfg::display::kCardWidth,
                    cfg::display::kCardHeight,
                    "TEMP",
                    &page_env_.temp,
                    &lv_font_montserrat_20);

    createValueCard(parent,
                    x1,
                    cfg::display::kCardStartY,
                    cfg::display::kCardWidth,
                    cfg::display::kCardHeight,
                    "HUMIDITY",
                    &page_env_.humidity,
                    &lv_font_montserrat_20);

    createValueCard(parent,
                    x0,
                    cfg::display::kCardSecondRowY,
                    cfg::display::kCardWidth,
                    cfg::display::kCardHeight,
                    "PRESSURE",
                    &page_env_.pressure,
                    &lv_font_montserrat_20);

    createValueCard(parent,
                    x1,
                    cfg::display::kCardSecondRowY,
                    cfg::display::kCardWidth,
                    cfg::display::kCardHeight,
                    "ALTITUDE",
                    &page_env_.altitude,
                    &lv_font_montserrat_20);

    page_env_.uptime_footer = lv_label_create(parent);
    lv_label_set_text(page_env_.uptime_footer, "Uptime: 00:00:00");
    lv_obj_set_style_text_color(page_env_.uptime_footer, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(page_env_.uptime_footer, &lv_font_montserrat_12, 0);
    lv_obj_align(page_env_.uptime_footer, LV_ALIGN_BOTTOM_MID, 0, -4);
}

void UiController::buildPageAqi(lv_obj_t *parent)
{
    createHeader(parent, "02 / GAS", 1);

    battery_labels_[1] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[1], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[1], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[1], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[1], LV_ALIGN_TOP_MID, 0, 8);

    page_aqi_.gas_arc = lv_arc_create(parent);
    lv_obj_set_size(page_aqi_.gas_arc, 112, 112);
    lv_obj_align(page_aqi_.gas_arc, LV_ALIGN_TOP_MID, 0, 30);
    lv_arc_set_rotation(page_aqi_.gas_arc, 135);
    lv_arc_set_bg_angles(page_aqi_.gas_arc, 0, 270);
    lv_arc_set_range(page_aqi_.gas_arc, static_cast<int32_t>(cfg::sensor::kGasGaugeMinKohm), static_cast<int32_t>(cfg::sensor::kGasGaugeMaxKohm));
    lv_arc_set_value(page_aqi_.gas_arc, 0);
    lv_obj_set_style_arc_width(page_aqi_.gas_arc, 9, LV_PART_MAIN);
    lv_obj_set_style_arc_width(page_aqi_.gas_arc, 9, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(page_aqi_.gas_arc, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
    lv_obj_set_style_arc_color(page_aqi_.gas_arc, lv_color_hex(cfg::color::kStatusOk), LV_PART_INDICATOR);
    lv_obj_remove_style(page_aqi_.gas_arc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(page_aqi_.gas_arc, LV_OBJ_FLAG_CLICKABLE);

    page_aqi_.gas_title = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_title, "Gas Resolution");
    lv_obj_set_width(page_aqi_.gas_title, 140);
    lv_obj_set_style_text_align(page_aqi_.gas_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(page_aqi_.gas_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(page_aqi_.gas_title, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_align_to(page_aqi_.gas_title, page_aqi_.gas_arc, LV_ALIGN_CENTER, 0, -10);

    page_aqi_.gas_value = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_value, "-- kOhm");
    lv_obj_set_width(page_aqi_.gas_value, 112);
    lv_obj_set_style_text_align(page_aqi_.gas_value, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(page_aqi_.gas_value, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(page_aqi_.gas_value, lv_color_hex(cfg::color::kTextPrimary), 0);
    lv_obj_align_to(page_aqi_.gas_value, page_aqi_.gas_arc, LV_ALIGN_CENTER, 0, 14);

    page_aqi_.gas_status_value = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_status_value, "No Data");
    lv_obj_set_width(page_aqi_.gas_status_value, 160);
    lv_obj_set_style_text_align(page_aqi_.gas_status_value, LV_TEXT_ALIGN_LEFT, 0);
    lv_obj_set_style_text_font(page_aqi_.gas_status_value, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_color(page_aqi_.gas_status_value, lv_color_hex(cfg::color::kTextPrimary), 0);
    lv_obj_align(page_aqi_.gas_status_value, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    page_aqi_.gas_trend_title = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_trend_title, "Gas Trend 5m");
    lv_obj_set_width(page_aqi_.gas_trend_title, 120);
    lv_obj_set_style_text_align(page_aqi_.gas_trend_title, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(page_aqi_.gas_trend_title, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(page_aqi_.gas_trend_title, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_align(page_aqi_.gas_trend_title, LV_ALIGN_BOTTOM_RIGHT, -10, -40);

    page_aqi_.gas_trend_value = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_trend_value, "Stable");
    lv_obj_set_width(page_aqi_.gas_trend_value, 120);
    lv_obj_set_style_text_align(page_aqi_.gas_trend_value, LV_TEXT_ALIGN_RIGHT, 0);
    lv_obj_set_style_text_font(page_aqi_.gas_trend_value, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_color(page_aqi_.gas_trend_value, lv_color_hex(cfg::color::kBootChecking), 0);
    lv_obj_align(page_aqi_.gas_trend_value, LV_ALIGN_BOTTOM_RIGHT, -10, -12);
}

void UiController::buildPageOutdoors(lv_obj_t *parent)
{
    bool wifi_connected = false;
    if (WiFiManager::instance().isInitialized())
    {
        wifi_connected = WiFiManager::instance().isConnected();
    }
    const char *page_title = "04 / OUTDOOR";
    createHeader(parent, page_title, 3);

    battery_labels_[3] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[3], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[3], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[3], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[3], LV_ALIGN_TOP_MID, 0, 8);

    page_outdoor_.no_wifi = lv_label_create(parent);
    lv_label_set_text(page_outdoor_.no_wifi, "WiFi Required");
    lv_obj_set_style_text_color(page_outdoor_.no_wifi, lv_color_hex(cfg::color::kBootChecking), 0);
    lv_obj_set_style_text_font(page_outdoor_.no_wifi, &lv_font_montserrat_14, 0);
    lv_obj_align(page_outdoor_.no_wifi, LV_ALIGN_CENTER, 0, 0);

    const lv_coord_t card_w = 148;
    const lv_coord_t card_h = 34;
    const lv_coord_t left_x = cfg::display::kMarginLeft;
    const lv_coord_t right_x = left_x + card_w + cfg::display::kCardGap;
    const lv_coord_t row0_y = 34;
    const lv_coord_t row1_y = 88;
    const lv_coord_t row2_y = 132;

    lv_obj_t *weather_card = createValueCard(parent,
                                             left_x,
                                             row0_y,
                                             cfg::display::kWidth - (cfg::display::kMarginLeft * 2),
                                             46,
                                             "WEATHER",
                                             &page_outdoor_.weather_status,
                                             &lv_font_montserrat_18);
    (void)weather_card;

    page_outdoor_.datetime = lv_label_create(weather_card);
    lv_label_set_text(page_outdoor_.datetime, "--:--");
    lv_obj_set_style_text_font(page_outdoor_.datetime, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(page_outdoor_.datetime, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_align(page_outdoor_.datetime, LV_ALIGN_TOP_RIGHT, 0, 0);

    lv_obj_t *temp_card = createSingleLineCard(parent,
                                               left_x,
                                               row1_y,
                                               card_w,
                                               card_h,
                                               "Temp: -- C",
                                               &page_outdoor_.temp,
                                               &lv_font_montserrat_12);
    (void)temp_card;

    lv_obj_t *humidity_card = createSingleLineCard(parent,
                                                   right_x,
                                                   row1_y,
                                                   card_w,
                                                   card_h,
                                                   "Hum: --%",
                                                   &page_outdoor_.humidity,
                                                   &lv_font_montserrat_12);
    (void)humidity_card;

    lv_obj_t *rain_card = createSingleLineCard(parent,
                                               left_x,
                                               row2_y,
                                               card_w,
                                               card_h,
                                               "Rain: -- mm",
                                               &page_outdoor_.rain,
                                               &lv_font_montserrat_12);
    (void)rain_card;

    lv_obj_t *clouds_card = createSingleLineCard(parent,
                                                 right_x,
                                                 row2_y,
                                                 card_w,
                                                 card_h,
                                                 "Clouds: --%",
                                                 &page_outdoor_.clouds,
                                                 &lv_font_montserrat_12);
    (void)clouds_card;

    if (!wifi_connected)
    {
        lv_obj_add_flag(page_outdoor_.datetime, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_outdoor_.weather_status, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_outdoor_.temp, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_outdoor_.humidity, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_outdoor_.rain, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(page_outdoor_.clouds, LV_OBJ_FLAG_HIDDEN);
    }

    if (wifi_connected)
    {
        lv_obj_add_flag(page_outdoor_.no_wifi, LV_OBJ_FLAG_HIDDEN);
    }
}

void UiController::buildPageSys(lv_obj_t *parent)
{
    createHeader(parent, "03 / SYSTEM", 2);

    battery_labels_[2] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[2], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[2], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[2], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[2], LV_ALIGN_TOP_MID, 0, 8);

    page_sys_.uptime = lv_label_create(parent);
    lv_label_set_text(page_sys_.uptime, "00:00:00");
    lv_obj_set_style_text_color(page_sys_.uptime, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(page_sys_.uptime, &lv_font_montserrat_22, 0);
    lv_obj_align(page_sys_.uptime, LV_ALIGN_TOP_MID, 0, 30);

    const lv_coord_t card_w = 148;
    const lv_coord_t card_h = 46;
    const lv_coord_t left_x = cfg::display::kMarginLeft;
    const lv_coord_t right_x = cfg::display::kMarginLeft + card_w + cfg::display::kCardGap;
    const lv_coord_t row1_y = 64;
    const lv_coord_t row2_y = 108;

    lv_obj_t *fetch_card = createValueCard(parent,
                                           left_x,
                                           row2_y,
                                           card_w,
                                           card_h,
                                           "LAST FETCH",
                                           &page_sys_.weather_update,
                                           &lv_font_montserrat_12);
    (void)fetch_card;

    lv_obj_t *cpu_card = createValueCard(parent,
                                         left_x,
                                         row1_y,
                                         card_w,
                                         card_h,
                                         LV_SYMBOL_SETTINGS " CPU Load",
                                         &page_sys_.cpu_load,
                                         &lv_font_montserrat_12);
    (void)cpu_card;

    lv_obj_t *storage_card = createValueCard(parent,
                                             right_x,
                                             row1_y,
                                             card_w,
                                             card_h,
                                             LV_SYMBOL_DRIVE " Storage",
                                             &page_sys_.storage,
                                             &lv_font_montserrat_12);
    (void)storage_card;

    lv_obj_t *wifi_card = createValueCard(parent,
                                          right_x,
                                          row2_y,
                                          card_w,
                                          card_h,
                                          LV_SYMBOL_WIFI " WiFi Status",
                                          &page_sys_.wifi_status,
                                          &lv_font_montserrat_12);
    (void)wifi_card;

    lv_label_set_text(page_sys_.cpu_load, "0%");
    lv_label_set_text(page_sys_.storage, "--/-- MB");
    lv_label_set_text(page_sys_.weather_update, "--:--:--");
    lv_obj_set_style_text_color(page_sys_.weather_update, lv_color_hex(cfg::color::kTextDim), 0);
    lv_label_set_text(page_sys_.wifi_status, "Offline");
    lv_obj_set_style_text_color(page_sys_.wifi_status, lv_color_hex(cfg::color::kError), 0);
}

void UiController::setPage(uint8_t page_index)
{
    bool wifi_connected = false;
    if (WiFiManager::instance().isInitialized())
    {
        wifi_connected = WiFiManager::instance().isConnected();
    }
    const uint8_t max_page = wifi_connected ? 3U : 2U;

    if (page_index > max_page)
    {
        return;
    }

    current_page_ = page_index;
    for (uint8_t i = 0; i < 4U; ++i)
    {
        if (pages_[i] == nullptr)
        {
            continue;
        }

        if (i == current_page_)
        {
            lv_obj_clear_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
        }
    }
}

void UiController::buildPages()
{
    if (!lvgl_ready_)
    {
        return;
    }

    bool wifi_connected = false;
    if (WiFiManager::instance().isInitialized())
    {
        wifi_connected = WiFiManager::instance().isConnected();
    }

    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(root, lv_color_hex(cfg::color::kBackground), 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(root, UiController::gestureEventCallback, LV_EVENT_GESTURE, nullptr);

    for (uint8_t i = 0; i < 4U; ++i)
    {
        pages_[i] = lv_obj_create(root);
        lv_obj_set_size(pages_[i], cfg::display::kWidth, cfg::display::kHeight);
        lv_obj_set_pos(pages_[i], 0, 0);
        lv_obj_set_style_bg_opa(pages_[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(pages_[i], 0, 0);
        lv_obj_set_style_pad_all(pages_[i], 0, 0);
        lv_obj_clear_flag(pages_[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(pages_[i], LV_OBJ_FLAG_CLICKABLE);
    }

    buildPageEnv(pages_[0]);
    buildPageAqi(pages_[1]);
    buildPageSys(pages_[2]);
    buildPageOutdoors(pages_[3]);

    outdoor_page_enabled_ = wifi_connected;

    if (!wifi_connected)
    {
        lv_obj_add_flag(pages_[3], LV_OBJ_FLAG_HIDDEN);
    }

    setPage(0);
    lv_scr_load(root);
}

void UiController::displayFlushCallback(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    UiController::instance().onDisplayFlush(disp, area, color_p);
}

void UiController::touchReadCallback(lv_indev_drv_t * /*drv*/, lv_indev_data_t *data)
{
    UiController::instance().onTouchRead(data);
}

void UiController::gestureEventCallback(lv_event_t *event)
{
    UiController::instance().onGesture(event);
}

void UiController::onDisplayFlush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    const uint16_t width = static_cast<uint16_t>(area->x2 - area->x1 + 1);
    const uint16_t height = static_cast<uint16_t>(area->y2 - area->y1 + 1);

    tft_.startWrite();
    tft_.setAddrWindow(area->x1, area->y1, width, height);
    tft_.pushColors(reinterpret_cast<uint16_t *>(&color_p->full), width * height, true);
    tft_.endWrite();

    lv_disp_flush_ready(disp);
}

void UiController::onTouchRead(lv_indev_data_t *data)
{
    if (!PowerManager::instance().isDisplayAwake() || !touch_hw_awake_)
    {
        data->state = LV_INDEV_STATE_RELEASED;
        return;
    }

    if (touch_ready_)
    {
        ScopedI2cBusLock lock(pdMS_TO_TICKS(12));
        if (!lock.ownsLock())
        {
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }

        const TouchPoints &touch_points = touch_.getTouchPoints();
        if (touch_points.hasPoints())
        {
            const TouchPoint &p = touch_points.getPoint(0);
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = static_cast<lv_coord_t>(clampI32(p.x, 0, cfg::display::kWidth - 1));
            data->point.y = static_cast<lv_coord_t>(clampI32(p.y, 0, cfg::display::kHeight - 1));
            PowerManager::instance().wakeFromInteraction();
            return;
        }
    }

    data->state = LV_INDEV_STATE_RELEASED;
}

void UiController::onGesture(lv_event_t *event)
{
    if (lv_event_get_code(event) != LV_EVENT_GESTURE)
    {
        return;
    }

    lv_indev_t *indev = lv_indev_get_act();
    if (indev == nullptr)
    {
        return;
    }

    const lv_dir_t direction = lv_indev_get_gesture_dir(indev);
    if (direction == LV_DIR_LEFT)
    {
        const uint8_t max_page = outdoor_page_enabled_ ? 3U : 2U;
        setPage((current_page_ + 1U) % (max_page + 1U));
    }
    else if (direction == LV_DIR_RIGHT)
    {
        const uint8_t max_page = outdoor_page_enabled_ ? 3U : 2U;
        setPage((current_page_ + max_page) % (max_page + 1U));
    }
}

uint32_t UiController::readBatteryMv(bool *has_battery)
{
    if (!battery_adc_ready_)
    {
        analogReadResolution(12);
        analogSetPinAttenuation(cfg::pins::kBatteryAdc, ADC_11db);
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &battery_adc_chars_);
        battery_adc_ready_ = true;
    }

    uint32_t raw_sum = 0;
    for (uint8_t i = 0; i < cfg::battery::kAdcSamples; ++i)
    {
        raw_sum += static_cast<uint32_t>(analogRead(cfg::pins::kBatteryAdc));
    }

    const uint32_t raw_avg = raw_sum / cfg::battery::kAdcSamples;
    const uint32_t mv = esp_adc_cal_raw_to_voltage(raw_avg, &battery_adc_chars_) * 2U;

    if (!bat_filter_ready_)
    {
        bat_fast_mv_ = mv;
        bat_slow_mv_ = mv;
        bat_filter_ready_ = true;
    }
    else
    {
        bat_fast_mv_ = (bat_fast_mv_ * 3U + mv) / 4U;
        bat_slow_mv_ = (bat_slow_mv_ * 15U + mv) / 16U;
    }

    const bool present = bat_fast_mv_ <= cfg::battery::kAbsentMv;
    if (has_battery != nullptr)
    {
        *has_battery = present;
    }

    return bat_fast_mv_;
}

void UiController::updateBatteryLabels()
{
    const uint32_t now_ms = millis();
    const bool do_sample = (bat_last_sample_ms_ == 0U) || (now_ms - bat_last_sample_ms_ >= cfg::battery::kEstimatorSampleMs);
    if (do_sample)
    {
        const uint32_t prev_sample_ms = bat_last_sample_ms_;
        bat_last_sample_ms_ = now_ms;

        bool has_battery = false;
        const uint32_t measured_mv = readBatteryMv(&has_battery);
        bat_usb_power_ = !has_battery;

        if (bat_usb_power_)
        {
            bat_soc_estimate_pct_ = 100.0f;
            bat_soc_ready_ = true;
        }
        else
        {
            const bool display_awake = PowerManager::instance().isDisplayAwake();
            const uint32_t ocv_mv = estimateBatteryOcvMv(measured_mv, display_awake);
            const float measured_soc = static_cast<float>(batteryPercentFromMv(ocv_mv));

            float blend = display_awake ? cfg::battery::kSocBlendActive : cfg::battery::kSocBlendSleep;
            const uint32_t ripple_mv = (bat_fast_mv_ >= bat_slow_mv_) ? (bat_fast_mv_ - bat_slow_mv_) : (bat_slow_mv_ - bat_fast_mv_);
            if ((!display_awake) && (ripple_mv <= cfg::battery::kRestDeltaMv))
            {
                blend = cfg::battery::kSocBlendRest;
            }

            if (!bat_soc_ready_)
            {
                bat_soc_estimate_pct_ = measured_soc;
                bat_soc_ready_ = true;
            }
            else
            {
                const float fused_soc = bat_soc_estimate_pct_ + ((measured_soc - bat_soc_estimate_pct_) * blend);

                uint32_t elapsed_ms = cfg::battery::kEstimatorSampleMs;
                if (prev_sample_ms > 0U)
                {
                    elapsed_ms = (now_ms > prev_sample_ms) ? (now_ms - prev_sample_ms) : cfg::battery::kEstimatorSampleMs;
                }
                const float elapsed_minutes = static_cast<float>(elapsed_ms) / 60000.0f;
                const float max_drop = cfg::battery::kMaxDropPctPerMin * elapsed_minutes;
                const float max_rise = cfg::battery::kMaxRisePctPerMinIdle * elapsed_minutes;

                if (fused_soc < (bat_soc_estimate_pct_ - max_drop))
                {
                    bat_soc_estimate_pct_ -= max_drop;
                }
                else if (fused_soc > (bat_soc_estimate_pct_ + max_rise))
                {
                    bat_soc_estimate_pct_ += max_rise;
                }
                else
                {
                    bat_soc_estimate_pct_ = fused_soc;
                }
            }

            if (bat_soc_estimate_pct_ < 0.0f)
            {
                bat_soc_estimate_pct_ = 0.0f;
            }
            if (bat_soc_estimate_pct_ > 100.0f)
            {
                bat_soc_estimate_pct_ = 100.0f;
            }
        }
    }

    uint8_t shown_pct = 100U;
    if (!bat_usb_power_)
    {
        const int32_t rounded_soc = clampI32(static_cast<int32_t>(lroundf(bat_soc_estimate_pct_)), 0, 100);
        shown_pct = static_cast<uint8_t>(rounded_soc);
    }

    const bool force_refresh = (bat_last_label_ms_ == 0U) || (now_ms - bat_last_label_ms_ >= cfg::battery::kLabelRefreshMs);
    if (!force_refresh && (shown_pct == bat_shown_pct_))
    {
        return;
    }

    bat_shown_pct_ = shown_pct;
    bat_last_label_ms_ = now_ms;

    const lv_color_t text_color = batteryColorFromPercent(shown_pct, bat_usb_power_);
    for (uint8_t i = 0; i < 4U; ++i)
    {
        if (battery_labels_[i] == nullptr)
        {
            continue;
        }

        lv_obj_set_style_text_color(battery_labels_[i], text_color, 0);
        lv_label_set_text_fmt(battery_labels_[i], LV_SYMBOL_BATTERY_FULL " %u%%", shown_pct);
    }
}

void UiController::updateValues()
{
    const uint32_t now_ms = millis();

    if ((last_uptime_refresh_ms_ == 0U) || (now_ms - last_uptime_refresh_ms_ >= cfg::timing::kUptimeRefreshMs))
    {
        last_uptime_refresh_ms_ = now_ms;

        char uptime[24] = {0};
        const uint64_t uptime_seconds = static_cast<uint64_t>(esp_timer_get_time()) / 1000000ULL;
        formatUptime(uptime, sizeof(uptime), uptime_seconds);

        lv_label_set_text_fmt(page_env_.uptime_footer, "Uptime: %s", uptime);
        lv_label_set_text(page_sys_.uptime, uptime);
    }

    const bool sensor_pages_due = (last_ui_refresh_ms_ == 0U) ||
                                  (now_ms - last_ui_refresh_ms_ >= cfg::timing::kUiValuesRefreshMs);
    if (sensor_pages_due)
    {
        last_ui_refresh_ms_ = now_ms;

        SensorManager &sensor_manager = SensorManager::instance();
        const SensorData data = sensor_manager.getData();
        uint32_t data_age_ms = UINT32_MAX;
        if (data.last_update_ms != 0U)
        {
            data_age_ms = now_ms - data.last_update_ms;
        }
        const uint32_t hold_window_ms = cfg::timing::kSensorStaleReinitMs + cfg::timing::kSensorLinkProbeMs;
        const bool hold_recent_snapshot = (data.last_update_ms != 0U) && (data_age_ms <= hold_window_ms);

        const bool sensor_connected_now = sensor_manager.isRealtimeConnected();
        const lv_color_t header_color = sensor_connected_now ? lv_color_hex(cfg::color::kValue) : lv_color_hex(cfg::color::kError);
        for (uint8_t i = 0; i < 3U; ++i)
        {
            if (env_headers_[i] != nullptr)
            {
                lv_obj_set_style_text_color(env_headers_[i], header_color, 0);
            }
        }

        char text[64] = {0};

        if (data.valid || hold_recent_snapshot)
        {
            const bool env_values_due = (data.last_update_ms != 0U) && (data.last_update_ms != last_env_snapshot_ms_);
            if (env_values_due)
            {
                last_env_snapshot_ms_ = data.last_update_ms;

                snprintf(text, sizeof(text), "%.2f C", data.temperature_c);
                lv_label_set_text(page_env_.temp, text);
                lv_obj_set_style_text_color(page_env_.temp, lv_color_hex(tempValueColor(data.temperature_c)), 0);

                snprintf(text, sizeof(text), "%.2f %%", data.humidity_pct);
                lv_label_set_text(page_env_.humidity, text);
                lv_obj_set_style_text_color(page_env_.humidity, lv_color_hex(humidityValueColor(data.humidity_pct)), 0);

                snprintf(text, sizeof(text), "%.2f hPa", data.pressure_hpa);
                lv_label_set_text(page_env_.pressure, text);

                snprintf(text, sizeof(text), "%.1f m", data.altitude_m);
                lv_label_set_text(page_env_.altitude, text);
            }

            const float gas_kohm = data.gas_resistance_kohm;
            const uint32_t gas_color = gasValueColor(gas_kohm);
            const int32_t gas_arc_value = clampI32(roundToInt(gas_kohm),
                                                   static_cast<int32_t>(cfg::sensor::kGasGaugeMinKohm),
                                                   static_cast<int32_t>(cfg::sensor::kGasGaugeMaxKohm));
            if (isfinite(gas_kohm))
            {
                const int32_t gas_kohm_int = roundToInt(gas_kohm);
                snprintf(text, sizeof(text), "%ld kOhm", static_cast<long>(gas_kohm_int));
            }
            else
            {
                snprintf(text, sizeof(text), "-- kOhm");
            }
            lv_label_set_text(page_aqi_.gas_value, text);
            lv_obj_set_style_text_color(page_aqi_.gas_value, lv_color_hex(gas_color), 0);
            lv_arc_set_value(page_aqi_.gas_arc, gas_arc_value);
            lv_obj_set_style_arc_color(page_aqi_.gas_arc, lv_color_hex(gas_color), LV_PART_INDICATOR);

            lv_label_set_text(page_aqi_.gas_status_value, gasStatusText(gas_kohm));
            lv_obj_set_style_text_color(page_aqi_.gas_status_value, lv_color_hex(gas_color), 0);

            lv_label_set_text(page_aqi_.gas_trend_value, gasTrendText(data.gas_trend_5m));
            lv_obj_set_style_text_color(page_aqi_.gas_trend_value, lv_color_hex(gasTrendColor(data.gas_trend_5m)), 0);
        }
        else
        {
            last_env_snapshot_ms_ = 0;
            lv_label_set_text(page_env_.temp, "--.- C");
            lv_obj_set_style_text_color(page_env_.temp, lv_color_hex(cfg::color::kValue), 0);
            lv_label_set_text(page_env_.humidity, "--.- %");
            lv_obj_set_style_text_color(page_env_.humidity, lv_color_hex(cfg::color::kValue), 0);
            lv_label_set_text(page_env_.pressure, "----.-- hPa");
            lv_label_set_text(page_env_.altitude, "---.- m");

            lv_label_set_text(page_aqi_.gas_value, "-- kOhm");
            lv_obj_set_style_text_color(page_aqi_.gas_value, lv_color_hex(cfg::color::kTextDim), 0);
            lv_label_set_text(page_aqi_.gas_status_value, "No Data");
            lv_obj_set_style_text_color(page_aqi_.gas_status_value, lv_color_hex(cfg::color::kTextDim), 0);
            lv_label_set_text(page_aqi_.gas_trend_value, "Stable");
            lv_obj_set_style_text_color(page_aqi_.gas_trend_value, lv_color_hex(cfg::color::kTextDim), 0);
            lv_obj_set_style_arc_color(page_aqi_.gas_arc, lv_color_hex(cfg::color::kBootChecking), LV_PART_INDICATOR);
            lv_arc_set_value(page_aqi_.gas_arc, 0);
        }
    }

    {
        WiFiManager &wifi = WiFiManager::instance();
        if ((last_wifi_status_refresh_ms_ == 0U) || (now_ms - last_wifi_status_refresh_ms_ >= cfg::timing::kWifiStatusRefreshMs))
        {
            last_wifi_status_refresh_ms_ = now_ms;
            const bool wifi_connected = wifi.isConnected();
            lv_label_set_text(page_sys_.wifi_status, wifi_connected ? "Connected" : "Offline");
            lv_obj_set_style_text_color(page_sys_.wifi_status,
                                        lv_color_hex(wifi_connected ? cfg::color::kStatusOk : cfg::color::kError),
                                        0);
        }

        if ((last_fetch_label_refresh_ms_ == 0U) || (now_ms - last_fetch_label_refresh_ms_ >= cfg::timing::kLastFetchLabelRefreshMs))
        {
            last_fetch_label_refresh_ms_ = now_ms;
            const WeatherSnapshot weather = wifi.getSnapshot();
            if ((page_sys_.weather_update == nullptr) || !weather.valid || (weather.last_update_ms == 0U) || (weather.last_update_epoch_utc == 0U) || !wifi.hasFreshWeather(now_ms))
            {
                if (page_sys_.weather_update != nullptr)
                {
                    lv_label_set_text(page_sys_.weather_update, "--:--:--");
                    lv_obj_set_style_text_color(page_sys_.weather_update, lv_color_hex(cfg::color::kTextDim), 0);
                }
            }
            else
            {
                const uint32_t day_seconds = 24UL * 60UL * 60UL;
                const uint32_t wib_offset_seconds = 7UL * 60UL * 60UL;
                const uint32_t fetch_seconds = (weather.last_update_epoch_utc + wib_offset_seconds) % day_seconds;
                const uint32_t hh = fetch_seconds / 3600UL;
                const uint32_t mm = (fetch_seconds % 3600UL) / 60UL;
                const uint32_t ss = fetch_seconds % 60UL;
                lv_label_set_text_fmt(page_sys_.weather_update,
                                      "%02lu:%02lu:%02lu",
                                      static_cast<unsigned long>(hh),
                                      static_cast<unsigned long>(mm),
                                      static_cast<unsigned long>(ss));
                lv_obj_set_style_text_color(page_sys_.weather_update,
                                            lv_color_hex(wifi.isOnlineMode() ? cfg::color::kValue : cfg::color::kBootChecking),
                                            0);
            }
        }
    }

    if ((last_cpu_refresh_ms_ == 0U) || (now_ms - last_cpu_refresh_ms_ >= cfg::timing::kCpuLoadRefreshMs))
    {
        last_cpu_refresh_ms_ = now_ms;
        const uint8_t load = estimateUiTaskLoadPercent();
        lv_label_set_text_fmt(page_sys_.cpu_load, "%u%%", load);
    }

    if ((last_sys_info_refresh_ms_ == 0U) || (now_ms - last_sys_info_refresh_ms_ >= cfg::timing::kSysInfoRefreshMs))
    {
        last_sys_info_refresh_ms_ = now_ms;

        const uint32_t storage_total = ESP.getFlashChipSize();
        const uint32_t storage_free = ESP.getFreeSketchSpace();
        const uint32_t bytes_per_mb = 1024UL * 1024UL;

        uint32_t free_tenths = 0;
        uint32_t total_tenths = 0;
        if (storage_total > 0U)
        {
            free_tenths = static_cast<uint32_t>((static_cast<uint64_t>(storage_free) * 10ULL) / bytes_per_mb);
            total_tenths = static_cast<uint32_t>((static_cast<uint64_t>(storage_total) * 10ULL) / bytes_per_mb);
        }

        lv_label_set_text_fmt(page_sys_.storage,
                              "%lu.%lu/%lu.%lu MB",
                              static_cast<unsigned long>(free_tenths / 10U),
                              static_cast<unsigned long>(free_tenths % 10U),
                              static_cast<unsigned long>(total_tenths / 10U),
                              static_cast<unsigned long>(total_tenths % 10U));
    }

    if ((outdoor_page_enabled_) && (now_ms - last_outdoor_refresh_ms_ >= cfg::timing::kUiValuesRefreshMs))
    {
        last_outdoor_refresh_ms_ = now_ms;

        WiFiManager &wifi = WiFiManager::instance();
        const WeatherSnapshot weather = wifi.getSnapshot();
        if (outdoor_page_enabled_ && weather.valid)
        {
            lv_label_set_text(page_outdoor_.weather_status, weatherConditionText(weather.weather_code));
            lv_obj_set_style_text_color(page_outdoor_.weather_status, lv_color_hex(weatherConditionColor(weather.weather_code)), 0);

            char temp_text[32] = {0};
            if (isfinite(weather.temperature_c))
            {
                snprintf(temp_text, sizeof(temp_text), "Temp: %.1f C", weather.temperature_c);
            }
            else
            {
                snprintf(temp_text, sizeof(temp_text), "Temp: -- C");
            }
            lv_label_set_text(page_outdoor_.temp, temp_text);
            lv_obj_set_style_text_color(page_outdoor_.temp, lv_color_hex(cfg::color::kValue), 0);

            char humidity_text[32] = {0};
            if (isfinite(weather.humidity_pct))
            {
                snprintf(humidity_text, sizeof(humidity_text), "Hum: %.0f%%", weather.humidity_pct);
            }
            else
            {
                snprintf(humidity_text, sizeof(humidity_text), "Hum: --%%");
            }
            lv_label_set_text(page_outdoor_.humidity, humidity_text);
            lv_obj_set_style_text_color(page_outdoor_.humidity, lv_color_hex(cfg::color::kValue), 0);

            if (weather.forecast_time[0] != '\0')
            {
                lv_label_set_text(page_outdoor_.datetime, weather.forecast_time);
                lv_obj_set_style_text_color(page_outdoor_.datetime, lv_color_hex(cfg::color::kValue), 0);
            }
            else
            {
                lv_label_set_text(page_outdoor_.datetime, "--:--");
                lv_obj_set_style_text_color(page_outdoor_.datetime, lv_color_hex(cfg::color::kTextDim), 0);
            }

            char rain_text[16] = {0};
            snprintf(rain_text, sizeof(rain_text), "Rain: %.1f mm", weather.precipitation_mm);
            lv_label_set_text(page_outdoor_.rain, rain_text);
            lv_obj_set_style_text_color(page_outdoor_.rain, lv_color_hex(rainAmountColor(weather.precipitation_mm)), 0);

            char cloud_text[16] = {0};
            snprintf(cloud_text, sizeof(cloud_text), "Clouds: %u%%", weather.cloud_coverage_pct);
            lv_label_set_text(page_outdoor_.clouds, cloud_text);
            lv_obj_set_style_text_color(page_outdoor_.clouds, lv_color_hex(cfg::color::kValue), 0);

            if (weather.weather_desc[0] != '\0')
            {
                lv_label_set_text(page_outdoor_.weather_status, weather.weather_desc);
                lv_obj_set_style_text_color(page_outdoor_.weather_status, lv_color_hex(cfg::color::kValue), 0);
            }
        }
    }
}

void UiController::taskEntry(void *parameter)
{
    auto *self = static_cast<UiController *>(parameter);
    if (self == nullptr)
    {
        self = &UiController::instance();
    }
    self->taskLoop();
}

void UiController::taskLoop()
{
    for (;;)
    {
        if (!lvgl_ready_)
        {
            vTaskDelay(pdMS_TO_TICKS(cfg::timing::kUiTaskActiveDelayMs));
            continue;
        }

        const bool display_awake = PowerManager::instance().isDisplayAwake();
        const uint32_t task_delay_ms = display_awake ? cfg::timing::kUiTaskActiveDelayMs : cfg::timing::kUiTaskSleepDelayMs;
        const uint64_t cycle_start_us = esp_timer_get_time();

        if (display_awake != prev_display_awake_)
        {
            syncTouchPowerState(display_awake);
            prev_display_awake_ = display_awake;
        }

        lv_tick_inc(task_delay_ms);
        lv_timer_handler();

        if (display_awake)
        {
            const uint32_t now_ms = millis();
            bool wifi_connected = false;
            if (WiFiManager::instance().isInitialized())
            {
                wifi_connected = WiFiManager::instance().isConnected();
            }

            if (wifi_connected != outdoor_page_enabled_)
            {
                outdoor_page_enabled_ = wifi_connected;
                const uint8_t max_page = outdoor_page_enabled_ ? 3U : 2U;

                if (current_page_ > max_page)
                {
                    current_page_ = 0;
                }

                for (uint8_t i = 0; i < 4U; ++i)
                {
                    if (pages_[i] == nullptr)
                    {
                        continue;
                    }
                    if (i == current_page_)
                    {
                        lv_obj_clear_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
                    }
                    else
                    {
                        lv_obj_add_flag(pages_[i], LV_OBJ_FLAG_HIDDEN);
                    }
                }
            }

            updateValues();
        }
        else
        {
            const uint32_t now_ms = millis();
            if ((last_bg_ui_refresh_ms_ == 0U) || (now_ms - last_bg_ui_refresh_ms_ >= cfg::timing::kUiBackgroundRefreshMs))
            {
                last_bg_ui_refresh_ms_ = now_ms;
                updateValues();
            }
        }

        updateBatteryLabels();

        const uint64_t busy_us = esp_timer_get_time() - cycle_start_us;
        const uint64_t scheduled_us = static_cast<uint64_t>(task_delay_ms) * 1000ULL;
        const uint64_t cycle_total_us = busy_us + scheduled_us;
        if (cycle_total_us > 0ULL)
        {
            float instant = (static_cast<float>(busy_us) * 100.0f) / static_cast<float>(cycle_total_us);
            if (display_awake && (instant < 1.0f))
            {
                instant = 1.0f;
            }
            cpu_load_estimate_pct_ = (cpu_load_estimate_pct_ * 0.80f) + (instant * 0.20f);
        }

        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}
