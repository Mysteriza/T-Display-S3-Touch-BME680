#include "ui_controller.h"

#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>

#include "config.h"
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

void UiController::formatUptime(char *out, size_t out_len, uint32_t uptime_seconds) const
{
    const uint32_t hours = uptime_seconds / 3600U;
    const uint32_t minutes = (uptime_seconds % 3600U) / 60U;
    const uint32_t seconds = uptime_seconds % 60U;
    snprintf(out, out_len, "%02lu:%02lu:%02lu", hours, minutes, seconds);
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

uint8_t UiController::estimateCpuLoadPercent(uint32_t now_ms)
{
    (void)now_ms;
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

UiController::IaqDescriptor UiController::describeIaqStatus(int32_t iaq_value) const
{
    const int32_t bounded_iaq = clampI32(iaq_value, 0, 500);

    uint32_t status_color = cfg::color::kError;
    const char *status = "Hazardous";
    if (bounded_iaq <= 50)
    {
        status = "Excellent";
        status_color = cfg::color::kStatusOk;
    }
    else if (bounded_iaq <= 100)
    {
        status = "Good";
        status_color = cfg::color::kStatusOk;
    }
    else if (bounded_iaq <= 150)
    {
        status = "Moderate";
        status_color = cfg::color::kBootChecking;
    }
    else if (bounded_iaq <= 200)
    {
        status = "Poor";
        status_color = cfg::color::kError;
    }
    else if (bounded_iaq <= 300)
    {
        status = "Unhealthy";
        status_color = cfg::color::kError;
    }

    return {status, status_color};
}

bool UiController::probeTouchAddress(uint8_t address)
{
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
    Wire.begin();
    Wire.setClock(400000);

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
    touch_ready_ = touch_.begin(Wire, touch_addr_, -1, -1);
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
    const uint32_t status_color = done ? (ok ? cfg::color::kStatusOk : cfg::color::kError) : cfg::color::kBootChecking;
    const char *status_text = done ? (ok ? "OK" : "Fail") : "Checking";

    char line[96] = {0};
    snprintf(line, sizeof(line), "%s #%.6X [%s]#", name, status_color, status_text);
    lv_label_set_recolor(label, true);
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
    lv_label_set_text(boot_title_, "SYSTEM SELF-CHECK");
    lv_obj_set_style_text_color(boot_title_, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(boot_title_, &lv_font_montserrat_22, 0);
    lv_obj_align(boot_title_, LV_ALIGN_TOP_MID, 0, 12);

    boot_lcd_ = lv_label_create(screen);
    boot_touch_ = lv_label_create(screen);
    boot_sensor_ = lv_label_create(screen);
    boot_data_ = lv_label_create(screen);
    boot_iaq_ = lv_label_create(screen);

    lv_obj_set_style_text_font(boot_lcd_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(boot_touch_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(boot_sensor_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(boot_data_, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_font(boot_iaq_, &lv_font_montserrat_14, 0);

    lv_obj_align(boot_lcd_, LV_ALIGN_TOP_LEFT, 16, 44);
    lv_obj_align(boot_touch_, LV_ALIGN_TOP_LEFT, 16, 64);
    lv_obj_align(boot_sensor_, LV_ALIGN_TOP_LEFT, 16, 84);
    lv_obj_align(boot_data_, LV_ALIGN_TOP_LEFT, 16, 104);
    lv_obj_align(boot_iaq_, LV_ALIGN_TOP_LEFT, 16, 124);

    lv_scr_load(screen);
}

void UiController::bootDiagUpdate(const BootDiagStatus &status)
{
    if (!lvgl_ready_ || (boot_lcd_ == nullptr))
    {
        return;
    }

    setBootLine(boot_lcd_, "LCD Initializing...", status.lcd_done, status.lcd_ok);
    setBootLine(boot_touch_, "Touch Controller...", status.touch_done, status.touch_ok);
    setBootLine(boot_sensor_, "BME680 Sensor...", status.sensor_done, status.sensor_ok);
    setBootLine(boot_data_, "Data Pipeline...", status.data_done, status.data_ok);
    setBootLine(boot_iaq_, "IAQ Engine...", status.iaq_done, status.iaq_ok);

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
    lv_label_set_text(left, LV_SYMBOL_DIRECTORY " Env_monitor");
    lv_obj_set_style_text_color(left, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(left, &lv_font_montserrat_14, 0);
    lv_obj_align(left, LV_ALIGN_TOP_LEFT, cfg::display::kMarginLeft, cfg::display::kHeaderY);

    lv_obj_t *right = lv_label_create(parent);
    lv_label_set_text(right, page_info);
    lv_obj_set_style_text_color(right, lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(right, &lv_font_montserrat_12, 0);
    lv_obj_align(right, LV_ALIGN_TOP_RIGHT, -cfg::display::kMarginRight, cfg::display::kHeaderRightY);

    if (page_index < 3U)
    {
        env_headers_[page_index] = left;
    }

    return left;
}

lv_obj_t *UiController::createValueCard(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                                        const char *title, lv_obj_t **value_label, const lv_font_t *value_font)
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
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

    *value_label = lv_label_create(card);
    lv_label_set_text(*value_label, "--");
    lv_obj_set_style_text_color(*value_label, lv_color_hex(cfg::color::kValue), 0);
    lv_obj_set_style_text_font(*value_label, value_font, 0);
    lv_obj_align(*value_label, LV_ALIGN_BOTTOM_LEFT, 0, 2);

    return card;
}

void UiController::buildPageEnv(lv_obj_t *parent)
{
    createHeader(parent, "PAGE 01 / ENV", 0);

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
    createHeader(parent, "PAGE 02 / AQI", 1);

    battery_labels_[1] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[1], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[1], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[1], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[1], LV_ALIGN_TOP_MID, 0, 8);

    page_aqi_.gas = lv_label_create(parent);
    lv_label_set_text(page_aqi_.gas, "Gas res.\n-- kOhm");
    lv_obj_set_style_text_color(page_aqi_.gas, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(page_aqi_.gas, &lv_font_montserrat_14, 0);
    lv_obj_align(page_aqi_.gas, LV_ALIGN_LEFT_MID, 8, -24);

    page_aqi_.accuracy = lv_label_create(parent);
    lv_label_set_text(page_aqi_.accuracy, "Accuracy\n-");
    lv_obj_set_style_text_color(page_aqi_.accuracy, lv_color_hex(cfg::color::kBootChecking), 0);
    lv_obj_set_style_text_font(page_aqi_.accuracy, &lv_font_montserrat_14, 0);
    lv_obj_align(page_aqi_.accuracy, LV_ALIGN_LEFT_MID, 8, 34);

    page_aqi_.iaq_arc = lv_arc_create(parent);
    lv_obj_set_size(page_aqi_.iaq_arc, 132, 132);
    lv_obj_align(page_aqi_.iaq_arc, LV_ALIGN_CENTER, 0, 12);
    lv_arc_set_rotation(page_aqi_.iaq_arc, 135);
    lv_arc_set_bg_angles(page_aqi_.iaq_arc, 0, 270);
    lv_arc_set_range(page_aqi_.iaq_arc, 0, 500);
    lv_arc_set_value(page_aqi_.iaq_arc, 0);
    lv_obj_set_style_arc_width(page_aqi_.iaq_arc, 10, LV_PART_MAIN);
    lv_obj_set_style_arc_width(page_aqi_.iaq_arc, 10, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(page_aqi_.iaq_arc, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
    lv_obj_set_style_arc_color(page_aqi_.iaq_arc, lv_color_hex(cfg::color::kStatusOk), LV_PART_INDICATOR);
    lv_obj_remove_style(page_aqi_.iaq_arc, nullptr, LV_PART_KNOB);
    lv_obj_clear_flag(page_aqi_.iaq_arc, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *iaq_label = lv_label_create(parent);
    lv_label_set_text(iaq_label, "IAQ");
    lv_obj_set_style_text_font(iaq_label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(iaq_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_width(iaq_label, 64);
    lv_obj_set_style_text_align(iaq_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(iaq_label, page_aqi_.iaq_arc, LV_ALIGN_CENTER, 0, -18);

    page_aqi_.iaq_number = lv_label_create(parent);
    lv_label_set_text(page_aqi_.iaq_number, "0");
    lv_obj_set_style_text_font(page_aqi_.iaq_number, &lv_font_montserrat_32, 0);
    lv_obj_set_style_text_color(page_aqi_.iaq_number, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_width(page_aqi_.iaq_number, 96);
    lv_obj_set_style_text_align(page_aqi_.iaq_number, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align_to(page_aqi_.iaq_number, page_aqi_.iaq_arc, LV_ALIGN_CENTER, 0, 18);

    page_aqi_.iaq_status = lv_label_create(parent);
    lv_label_set_text(page_aqi_.iaq_status, "Status\n-");
    lv_obj_set_style_text_font(page_aqi_.iaq_status, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(page_aqi_.iaq_status, lv_color_hex(cfg::color::kStatusOk), 0);
    lv_obj_align(page_aqi_.iaq_status, LV_ALIGN_RIGHT_MID, -8, -8);
}

void UiController::buildPageSys(lv_obj_t *parent)
{
    createHeader(parent, "PAGE 03 / UPTIME", 2);

    battery_labels_[2] = lv_label_create(parent);
    lv_label_set_text(battery_labels_[2], LV_SYMBOL_BATTERY_EMPTY " --%");
    lv_obj_set_style_text_color(battery_labels_[2], lv_color_hex(cfg::color::kTextDim), 0);
    lv_obj_set_style_text_font(battery_labels_[2], &lv_font_montserrat_12, 0);
    lv_obj_align(battery_labels_[2], LV_ALIGN_TOP_MID, 0, 8);

    page_sys_.uptime = lv_label_create(parent);
    lv_label_set_text(page_sys_.uptime, "00:00:00");
    lv_obj_set_style_text_color(page_sys_.uptime, lv_color_hex(0xCCFFFF), 0);
    lv_obj_set_style_text_font(page_sys_.uptime, &lv_font_montserrat_36, 0);
    lv_obj_align(page_sys_.uptime, LV_ALIGN_TOP_MID, 0, 34);

    lv_obj_t *cpu_card = createValueCard(parent,
                                         cfg::display::kMarginLeft,
                                         98,
                                         cfg::display::kCardWidth,
                                         64,
                                         LV_SYMBOL_SETTINGS " CPU Load",
                                         &page_sys_.cpu_load,
                                         &lv_font_montserrat_22);
    (void)cpu_card;

    lv_obj_t *storage_card = createValueCard(parent,
                                             cfg::display::kMarginLeft + cfg::display::kCardWidth + cfg::display::kCardGap,
                                             98,
                                             cfg::display::kCardWidth,
                                             64,
                                             LV_SYMBOL_DRIVE " Storage",
                                             &page_sys_.storage,
                                             &lv_font_montserrat_18);
    (void)storage_card;

    lv_label_set_text(page_sys_.cpu_load, "0%");
    lv_label_set_text(page_sys_.storage, "--/-- MB");
}

void UiController::setPage(uint8_t page_index)
{
    if (page_index > 2U)
    {
        return;
    }

    current_page_ = page_index;
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
    lv_obj_add_event_cb(root, UiController::gestureEventCallback, LV_EVENT_GESTURE, nullptr);

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
        setPage((current_page_ + 1U) % 3U);
    }
    else if (direction == LV_DIR_RIGHT)
    {
        setPage((current_page_ + 2U) % 3U);
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
    for (uint8_t i = 0; i < 3U; ++i)
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

        char uptime[16] = {0};
        const uint64_t uptime_seconds = static_cast<uint64_t>(esp_timer_get_time()) / 1000000ULL;
        formatUptime(uptime, sizeof(uptime), static_cast<uint32_t>(uptime_seconds));

        lv_label_set_text_fmt(page_env_.uptime_footer, "Uptime: %s", uptime);
        lv_label_set_text(page_sys_.uptime, uptime);
    }

    if ((last_ui_refresh_ms_ != 0U) && (now_ms - last_ui_refresh_ms_ < cfg::timing::kUiValuesRefreshMs))
    {
        return;
    }
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
        snprintf(text, sizeof(text), "%.2f C", data.temperature_c);
        lv_label_set_text(page_env_.temp, text);

        snprintf(text, sizeof(text), "%.2f %%", data.humidity_pct);
        lv_label_set_text(page_env_.humidity, text);

        snprintf(text, sizeof(text), "%.2f hPa", data.pressure_hpa);
        lv_label_set_text(page_env_.pressure, text);

        snprintf(text, sizeof(text), "%.1f m", data.altitude_m);
        lv_label_set_text(page_env_.altitude, text);

        if (isfinite(data.gas_resistance_kohm))
        {
            const int32_t gas_kohm = roundToInt(data.gas_resistance_kohm);
            snprintf(text, sizeof(text), "%ld kOhm", static_cast<long>(gas_kohm));
        }
        else
        {
            snprintf(text, sizeof(text), "-- kOhm");
        }
        lv_label_set_text_fmt(page_aqi_.gas, "Gas res.\n%s", text);

        const uint8_t acc_level = (data.iaq_accuracy > 3U) ? 3U : data.iaq_accuracy;
        const char *acc_text = (acc_level == 0U) ? "Very Low" : (acc_level == 1U) ? "Low"
                                                            : (acc_level == 2U)   ? "Medium"
                                                                                  : "High";
        const char *model_text = (data.iaq_model_state == 2U)   ? "Adaptive"
                                 : (data.iaq_model_state == 1U) ? "Learning"
                                 : (data.iaq_model_state == 3U) ? "Fallback"
                                                                : "Baseline";
        uint32_t acc_color = cfg::color::kError;
        if (acc_level == 2U)
        {
            acc_color = cfg::color::kBootChecking;
        }
        else if (acc_level >= 3U)
        {
            acc_color = cfg::color::kStatusOk;
        }
        lv_label_set_text_fmt(page_aqi_.accuracy,
                              "Accuracy\n%s %u%%\n%s",
                              acc_text,
                              data.iaq_model_confidence,
                              model_text);
        lv_obj_set_style_text_color(page_aqi_.accuracy, lv_color_hex(acc_color), 0);

        float iaq_display = data.iaq;
        if (!isfinite(iaq_display) && isfinite(data.iaq_static))
        {
            iaq_display = data.iaq_static;
        }

        int32_t iaq_value = 0;
        if (isfinite(iaq_display))
        {
            const int32_t iaq_int = roundToInt(iaq_display);
            snprintf(text, sizeof(text), "%ld", static_cast<long>(iaq_int));
            lv_label_set_text(page_aqi_.iaq_number, text);
            iaq_value = iaq_int;
        }
        else
        {
            lv_label_set_text(page_aqi_.iaq_number, "--");
        }

        iaq_value = clampI32(iaq_value, 0, 500);

        int32_t iaq_effective = iaq_value;
        if (isfinite(data.iaq_effective))
        {
            iaq_effective = clampI32(roundToInt(data.iaq_effective), 0, 500);
        }
        lv_arc_set_value(page_aqi_.iaq_arc, iaq_effective);

        if (isfinite(iaq_display))
        {
            const IaqDescriptor descriptor = describeIaqStatus(iaq_effective);
            lv_label_set_text_fmt(page_aqi_.iaq_status, "Status\n%s", descriptor.status);
            lv_obj_set_style_arc_color(page_aqi_.iaq_arc, lv_color_hex(descriptor.status_color), LV_PART_INDICATOR);
            lv_obj_set_style_text_color(page_aqi_.iaq_status, lv_color_hex(descriptor.status_color), 0);
        }
        else
        {
            lv_label_set_text(page_aqi_.iaq_status, "Status\nNo Data");
            lv_obj_set_style_arc_color(page_aqi_.iaq_arc, lv_color_hex(cfg::color::kBootChecking), LV_PART_INDICATOR);
            lv_obj_set_style_text_color(page_aqi_.iaq_status, lv_color_hex(cfg::color::kBootChecking), 0);
        }
    }
    else
    {
        lv_label_set_text(page_env_.temp, "--.- C");
        lv_label_set_text(page_env_.humidity, "--.- %");
        lv_label_set_text(page_env_.pressure, "----.-- hPa");
        lv_label_set_text(page_env_.altitude, "---.- m");

        lv_label_set_text(page_aqi_.gas, "Gas res.\n-- kOhm");
        lv_label_set_text(page_aqi_.accuracy, "Accuracy\n-");
        lv_obj_set_style_text_color(page_aqi_.accuracy, lv_color_hex(cfg::color::kBootChecking), 0);
        lv_label_set_text(page_aqi_.iaq_number, "--");
        lv_label_set_text(page_aqi_.iaq_status, "Status\nNo Data");
        lv_obj_set_style_arc_color(page_aqi_.iaq_arc, lv_color_hex(cfg::color::kBootChecking), LV_PART_INDICATOR);
        lv_obj_set_style_text_color(page_aqi_.iaq_status, lv_color_hex(cfg::color::kBootChecking), 0);
        lv_arc_set_value(page_aqi_.iaq_arc, 0);
    }

    if ((last_cpu_refresh_ms_ == 0U) || (now_ms - last_cpu_refresh_ms_ >= cfg::timing::kCpuLoadRefreshMs))
    {
        last_cpu_refresh_ms_ = now_ms;
        const uint8_t load = estimateCpuLoadPercent(now_ms);
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
