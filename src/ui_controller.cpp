#include "ui_controller.h"

#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>

#include "config.h"
#include "i2c_bus_lock.h"
#include "power_mgmt.h"
#include "sensor_manager.h"

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

    for (uint8_t i = 0; i < 3U; ++i)
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

    lv_scr_load(screen);
}

void UiController::bootDiagUpdate(const BootDiagStatus &status)
{
    if (!lvgl_ready_)
    {
        return;
    }

    for (uint8_t i = 0; i < 3U; ++i)
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
    lv_obj_t *title = lv_label_create(parent);
    lv_label_set_text(title, page_info);
    lv_obj_set_style_text_color(title, lv_color_hex(cfg::color::kTextPrimary), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_12, 0);
    lv_obj_align(title, LV_ALIGN_BOTTOM_MID, 0, -8);

    return title;
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
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_all(card, 8, 0);

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
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(card, 12, 0);
    lv_obj_set_style_pad_left(card, 8, 0);
    lv_obj_set_style_pad_right(card, 8, 0);
    lv_obj_set_style_pad_top(card, 6, 0);
    lv_obj_set_style_pad_bottom(card, 6, 0);

    *text_label = lv_label_create(card);
    lv_label_set_text(*text_label, text);
    lv_obj_set_style_text_color(*text_label, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(*text_label, text_font, 0);
    lv_obj_align(*text_label, LV_ALIGN_LEFT_MID, 0, 0);

    return card;
}

void UiController::buildPageEnv(lv_obj_t *parent)
{
    // Top-left "ENV" label with sensor status indicator
    env_status_labels_[0] = lv_label_create(parent);
    lv_label_set_text(env_status_labels_[0], LV_SYMBOL_OK " ENV");
    lv_obj_set_style_text_color(env_status_labels_[0], lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(env_status_labels_[0], &lv_font_montserrat_12, 0);
    lv_obj_align(env_status_labels_[0], LV_ALIGN_TOP_LEFT, cfg::display::kMarginLeft, 6);

    createHeader(parent, "01 / ENV", 0);

    battery_labels_[0] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[0], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[0], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[0], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[0], LV_ALIGN_TOP_RIGHT, -8, 8);

    const lv_coord_t x0 = cfg::display::kMarginLeft;

    createValueCard(parent, x0, cfg::display::kCardStartY,
                    cfg::display::kCardWidth, cfg::display::kCardHeight,
                    "TEMPERATURE", &page_env_.temp, &lv_font_montserrat_22);

    createValueCard(parent, x0, cfg::display::kCardSecondRowY,
                    cfg::display::kCardWidth, cfg::display::kCardHeight,
                    "HUMIDITY", &page_env_.humidity, &lv_font_montserrat_22);

    createValueCard(parent, x0, cfg::display::kCardThirdRowY,
                    cfg::display::kCardWidth, cfg::display::kCardHeight,
                    "PRESSURE", &page_env_.pressure, &lv_font_montserrat_22);

    createValueCard(parent, x0, cfg::display::kCardFourthRowY,
                    cfg::display::kCardWidth, cfg::display::kCardHeight,
                    "ALTITUDE", &page_env_.altitude, &lv_font_montserrat_22);
}

void UiController::buildPageAqi(lv_obj_t *parent)
{
    // Top-left "ENV" label with sensor status indicator
    env_status_labels_[1] = lv_label_create(parent);
    lv_label_set_text(env_status_labels_[1], LV_SYMBOL_OK " ENV");
    lv_obj_set_style_text_color(env_status_labels_[1], lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(env_status_labels_[1], &lv_font_montserrat_12, 0);
    lv_obj_align(env_status_labels_[1], LV_ALIGN_TOP_LEFT, cfg::display::kMarginLeft, 6);

    createHeader(parent, "02 / GAS", 1);

    battery_labels_[1] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[1], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[1], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[1], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[1], LV_ALIGN_TOP_RIGHT, -8, 8);

    const lv_coord_t x0 = cfg::display::kMarginLeft;

    page_aqi_.gas_value = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_value, "--.- kOhm");
    lv_obj_set_style_text_color(page_aqi_.gas_value, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(page_aqi_.gas_value, &lv_font_montserrat_22, 0);
    lv_obj_align(page_aqi_.gas_value, LV_ALIGN_TOP_MID, 0, 40);

    page_aqi_.gas_status_value = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas_status_value, "INIT");
    lv_obj_set_style_text_color(page_aqi_.gas_status_value, lv_color_hex(cfg::color::kBootChecking), 0);
    lv_obj_set_style_text_font(page_aqi_.gas_status_value, &lv_font_montserrat_14, 0);
    lv_obj_align(page_aqi_.gas_status_value, LV_ALIGN_TOP_MID, 0, 72);

    createValueCard(parent, x0, cfg::display::kCardThirdRowY,
                    cfg::display::kCardWidth, cfg::display::kCardHeight,
                    "GAS TREND", &page_aqi_.gas_trend_value, &lv_font_montserrat_18);
}

void UiController::buildPageSys(lv_obj_t *parent)
{
    // Top-left "ENV" label with sensor status indicator
    env_status_labels_[2] = lv_label_create(parent);
    lv_label_set_text(env_status_labels_[2], LV_SYMBOL_OK " ENV");
    lv_obj_set_style_text_color(env_status_labels_[2], lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(env_status_labels_[2], &lv_font_montserrat_12, 0);
    lv_obj_align(env_status_labels_[2], LV_ALIGN_TOP_LEFT, cfg::display::kMarginLeft, 6);

    createHeader(parent, "03 / SYSTEM", 2);

    battery_labels_[2] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[2], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[2], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[2], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[2], LV_ALIGN_TOP_RIGHT, -8, 8);

    page_sys_.uptime = lv_label_create(parent);
    lv_label_set_text(page_sys_.uptime, "00:00:00");
    lv_obj_set_style_text_color(page_sys_.uptime, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(page_sys_.uptime, &lv_font_montserrat_22, 0);
    lv_obj_align(page_sys_.uptime, LV_ALIGN_TOP_MID, 0, 40);

    const lv_coord_t left_x = cfg::display::kMarginLeft;

    lv_obj_t *cpu_card = createValueCard(parent, left_x, 80,
                                         cfg::display::kCardWidth, cfg::display::kCardHeight,
                                         "CPU Load", &page_sys_.cpu_load, &lv_font_montserrat_18);
    (void)cpu_card;

    page_sys_.cpu_bar = lv_bar_create(parent);
    lv_obj_set_size(page_sys_.cpu_bar, 154, 6);
    lv_obj_set_pos(page_sys_.cpu_bar, left_x, 80 + cfg::display::kCardHeight + 4);
    lv_bar_set_range(page_sys_.cpu_bar, 0, 100);
    lv_bar_set_value(page_sys_.cpu_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(page_sys_.cpu_bar, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(page_sys_.cpu_bar, lv_color_hex(cfg::color::kValue), LV_PART_INDICATOR);
    lv_obj_set_style_radius(page_sys_.cpu_bar, 3, 0);

    lv_obj_t *mem_card = createValueCard(parent, left_x, 160,
                                         cfg::display::kCardWidth, cfg::display::kCardHeight,
                                         "Free Memory", &page_sys_.storage, &lv_font_montserrat_18);
    (void)mem_card;

    page_sys_.mem_bar = lv_bar_create(parent);
    lv_obj_set_size(page_sys_.mem_bar, 154, 6);
    lv_obj_set_pos(page_sys_.mem_bar, left_x, 160 + cfg::display::kCardHeight + 4);
    lv_bar_set_range(page_sys_.mem_bar, 0, 100);
    lv_bar_set_value(page_sys_.mem_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(page_sys_.mem_bar, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
    lv_obj_set_style_bg_color(page_sys_.mem_bar, lv_color_hex(cfg::color::kValue), LV_PART_INDICATOR);
    lv_obj_set_style_radius(page_sys_.mem_bar, 3, 0);
}

void UiController::setPage(uint8_t page_index)
{
    if (!lvgl_ready_)
    {
        return;
    }

    const uint8_t max_page = 2U;
    current_page_ = (page_index > max_page) ? 0 : page_index;

    for (uint8_t i = 0; i < 3U; ++i)
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

    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(root, lv_color_hex(cfg::color::kBackground), 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(root, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(root, UiController::gestureEventCallback, LV_EVENT_CLICKED, nullptr);

    for (uint8_t i = 0; i < 3U; ++i)
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

    setPage(0);
    lv_scr_load(root);

    updateBatteryLabels();
    updateValues();
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
    if (lv_event_get_code(event) != LV_EVENT_CLICKED)
    {
        return;
    }

    constexpr uint32_t kTapCooldownMs = 150U;
    const uint32_t now_ms = millis();
    if ((now_ms - last_gesture_ms_) < kTapCooldownMs)
    {
        return;
    }
    last_gesture_ms_ = now_ms;

    const uint8_t max_page = 2U;
    setPage((current_page_ + 1U) % (max_page + 1U));
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
    bool updated_soc = false;

    if ((bat_last_sample_ms_ == 0U) || (now_ms - bat_last_sample_ms_ >= cfg::battery::kEstimatorSampleMs))
    {
        bat_last_sample_ms_ = now_ms;
        bool has_battery = false;
        const uint32_t mv = readBatteryMv(&has_battery);

        if (!has_battery)
        {
            bat_soc_estimate_pct_ = 100.0f;
            bat_shown_pct_ = 100;
            bat_usb_power_ = true;
            updated_soc = true;
        }
        else
        {
            bat_usb_power_ = (mv >= 4250U);
            const uint32_t ocv_mv = estimateBatteryOcvMv(mv, prev_display_awake_);
            const uint8_t new_pct = batteryPercentFromMv(ocv_mv);

            if (!bat_soc_ready_)
            {
                bat_soc_estimate_pct_ = static_cast<float>(new_pct);
                bat_soc_ready_ = true;
            }
            else
            {
                float blend = bat_usb_power_ ? cfg::battery::kSocBlendCharging : (prev_display_awake_ ? cfg::battery::kSocBlendActive : cfg::battery::kSocBlendSleep);
                bat_soc_estimate_pct_ = (bat_soc_estimate_pct_ * (1.0f - blend)) + (static_cast<float>(new_pct) * blend);
            }

            float safe_soc = bat_soc_estimate_pct_;
            if (safe_soc < 0.0f)
                safe_soc = 0.0f;
            if (safe_soc > 100.0f)
                safe_soc = 100.0f;

            uint8_t current_pct = static_cast<uint8_t>(roundToInt(safe_soc));
            if (current_pct != bat_shown_pct_)
            {
                bat_shown_pct_ = current_pct;
                updated_soc = true;
            }
        }
    }

    if (updated_soc || (bat_last_label_ms_ == 0U) || (now_ms - bat_last_label_ms_ >= cfg::battery::kLabelRefreshMs))
    {
        bat_last_label_ms_ = now_ms;

        char buf[16];
        if (bat_usb_power_)
        {
            snprintf(buf, sizeof(buf), LV_SYMBOL_CHARGE " %u%%", bat_shown_pct_);
        }
        else
        {
            const char *bat_sym = LV_SYMBOL_BATTERY_FULL;
            if (bat_shown_pct_ <= 20)
                bat_sym = LV_SYMBOL_BATTERY_EMPTY;
            else if (bat_shown_pct_ <= 40)
                bat_sym = LV_SYMBOL_BATTERY_1;
            else if (bat_shown_pct_ <= 60)
                bat_sym = LV_SYMBOL_BATTERY_2;
            else if (bat_shown_pct_ <= 80)
                bat_sym = LV_SYMBOL_BATTERY_3;

            snprintf(buf, sizeof(buf), "%s %u%%", bat_sym, bat_shown_pct_);
        }

        const lv_color_t color = batteryColorFromPercent(bat_shown_pct_, bat_usb_power_);
        for (uint8_t i = 0; i < 3U; ++i)
        {
            if (battery_labels_[i] != nullptr)
            {
                lv_label_set_text(battery_labels_[i], buf);
                lv_obj_set_style_text_color(battery_labels_[i], color, 0);
            }
        }
    }
}

void UiController::updateEnvStatusLabels()
{
    const SensorData env = SensorManager::instance().getData();
    const bool sensor_connected = env.valid && env.last_update_ms != 0U;

    const uint32_t color = sensor_connected ? cfg::color::kValue : cfg::color::kError;
    const char *text = sensor_connected ? LV_SYMBOL_OK " ENV" : LV_SYMBOL_CLOSE " ENV";

    for (uint8_t i = 0; i < 3U; ++i)
    {
        if (env_status_labels_[i] != nullptr)
        {
            lv_label_set_text(env_status_labels_[i], text);
            lv_obj_set_style_text_color(env_status_labels_[i], lv_color_hex(color), 0);
        }
    }
}

void UiController::updateValues()
{
    const uint32_t now_ms = millis();
    const bool display_awake = PowerManager::instance().isDisplayAwake();
    const SensorData env = SensorManager::instance().getData();

    if ((last_env_snapshot_ms_ == 0U) || (now_ms - last_env_snapshot_ms_ >= cfg::timing::kUiValuesRefreshMs))
    {
        last_env_snapshot_ms_ = now_ms;

        if (env.valid)
        {
            if (page_env_.temp != nullptr)
            {
                lv_label_set_text_fmt(page_env_.temp, "%d.%d C", (int)env.temperature_c, abs((int)(env.temperature_c * 10) % 10));
                lv_obj_set_style_text_color(page_env_.temp, lv_color_hex(tempValueColor(env.temperature_c)), 0);
            }
            if (page_env_.humidity != nullptr)
            {
                lv_label_set_text_fmt(page_env_.humidity, "%d.%d %%", (int)env.humidity_pct, abs((int)(env.humidity_pct * 10) % 10));
                lv_obj_set_style_text_color(page_env_.humidity, lv_color_hex(cfg::color::kValue), 0);
            }
            if (page_env_.pressure != nullptr)
            {
                lv_label_set_text_fmt(page_env_.pressure, "%d hPa", (int)env.pressure_hpa);
                lv_obj_set_style_text_color(page_env_.pressure, lv_color_hex(cfg::color::kValue), 0);
            }

            if (page_env_.altitude != nullptr)
            {
                lv_label_set_text_fmt(page_env_.altitude, "%d m", (int)env.altitude_m);
                lv_obj_set_style_text_color(page_env_.altitude, lv_color_hex(cfg::color::kValue), 0);
            }

            if (page_aqi_.gas_value != nullptr)
            {
                if (env.gas_resistance_kohm < 1.0f)
                {
                    lv_label_set_text(page_aqi_.gas_value, "--.- kOhm");
                }
                else
                {
                    lv_label_set_text_fmt(page_aqi_.gas_value, "%.1f kOhm", env.gas_resistance_kohm);
                }
            }

            if (page_aqi_.gas_status_value != nullptr)
            {
                const char *status_text = "Good";
                uint32_t status_color = cfg::color::kStatusOk;

                if (env.gas_resistance_kohm < 20.0f)
                {
                    status_text = "Poor";
                    status_color = cfg::color::kError;
                }
                else if (env.gas_resistance_kohm < 50.0f)
                {
                    status_text = "Moderate";
                    status_color = cfg::color::kBootChecking;
                }

                lv_label_set_text(page_aqi_.gas_status_value, status_text);
                lv_obj_set_style_text_color(page_aqi_.gas_status_value, lv_color_hex(status_color), 0);
            }

            if (page_aqi_.gas_trend_value != nullptr)
            {
                const char *trend_text = "STABLE";
                uint32_t trend_color = cfg::color::kTextDim;
                if (env.gas_resistance_kohm > 0.0f)
                {
                    switch (env.gas_trend_5m)
                    {
                    case 1:
                        trend_text = "IMPROVING " LV_SYMBOL_UP;
                        trend_color = cfg::color::kStatusOk;
                        break;
                    case -1:
                        trend_text = "DEGRADING " LV_SYMBOL_DOWN;
                        trend_color = cfg::color::kError;
                        break;
                    default:
                        trend_text = "STABLE";
                        trend_color = cfg::color::kValue;
                        break;
                    }
                }

                lv_label_set_text(page_aqi_.gas_trend_value, trend_text);
                lv_obj_set_style_text_color(page_aqi_.gas_trend_value, lv_color_hex(trend_color), 0);
            }
        }
    }

    if (display_awake && ((last_uptime_refresh_ms_ == 0U) || (now_ms - last_uptime_refresh_ms_ >= cfg::timing::kUptimeRefreshMs)))
    {
        last_uptime_refresh_ms_ = now_ms;
        char uptime_str[32];
        formatUptime(uptime_str, sizeof(uptime_str), now_ms / 1000U);
        if (page_env_.uptime_footer != nullptr)
        {
            lv_label_set_text_fmt(page_env_.uptime_footer, "T-Display: %s", uptime_str);
        }
        if (page_sys_.uptime != nullptr)
        {
            lv_label_set_text(page_sys_.uptime, uptime_str);
        }
    }

    if (display_awake && ((last_cpu_refresh_ms_ == 0U) || (now_ms - last_cpu_refresh_ms_ >= cfg::timing::kCpuLoadRefreshMs)))
    {
        last_cpu_refresh_ms_ = now_ms;
        if (page_sys_.cpu_load != nullptr)
        {
            lv_label_set_text_fmt(page_sys_.cpu_load, "%d %%", (int)cpu_load_estimate_pct_);
        }
        if (page_sys_.cpu_bar != nullptr)
        {
            lv_bar_set_value(page_sys_.cpu_bar, static_cast<int32_t>(cpu_load_estimate_pct_), LV_ANIM_OFF);
            uint32_t bar_color = cfg::color::kValue;
            if (cpu_load_estimate_pct_ > 80.0f) bar_color = cfg::color::kError;
            else if (cpu_load_estimate_pct_ > 50.0f) bar_color = cfg::color::kBootChecking;
            lv_obj_set_style_bg_color(page_sys_.cpu_bar, lv_color_hex(bar_color), LV_PART_INDICATOR);
        }
    }

    if (display_awake && ((last_sys_info_refresh_ms_ == 0U) || (now_ms - last_sys_info_refresh_ms_ >= cfg::timing::kSysInfoRefreshMs)))
    {
        last_sys_info_refresh_ms_ = now_ms;
        if (page_sys_.storage != nullptr)
        {
            uint32_t free_heap_kb = esp_get_free_heap_size() / 1024U;
            lv_label_set_text_fmt(page_sys_.storage, "%lu KB", static_cast<unsigned long>(free_heap_kb));
        }
        if (page_sys_.mem_bar != nullptr)
        {
            uint32_t free_heap_kb = esp_get_free_heap_size() / 1024U;
            constexpr uint32_t kMaxHeapKb = 10000U;
            uint8_t free_pct = static_cast<uint8_t>((free_heap_kb * 100U) / kMaxHeapKb);
            free_pct = clampI32(free_pct, 0, 100);
            lv_bar_set_value(page_sys_.mem_bar, static_cast<int32_t>(free_pct), LV_ANIM_OFF);
            uint32_t bar_color = cfg::color::kValue;
            if (free_pct < 20) bar_color = cfg::color::kError;
            else if (free_pct < 40) bar_color = cfg::color::kBootChecking;
            lv_obj_set_style_bg_color(page_sys_.mem_bar, lv_color_hex(bar_color), LV_PART_INDICATOR);
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
            updateValues();
        }

        updateBatteryLabels();
        updateEnvStatusLabels();

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
