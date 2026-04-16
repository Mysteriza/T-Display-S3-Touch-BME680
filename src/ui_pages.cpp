#include "ui_pages.h"

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TouchDrv.hpp>
#include <Wire.h>
#include <esp_heap_caps.h>
#include <esp_adc_cal.h>
#include <esp_timer.h>
#include <lvgl.h>
#include <math.h>

#include "config.h"
#include "power_mgmt.h"
#include "sensors.h"

// Keep explicit declarations so editor diagnostics can resolve these symbols.
extern const lv_font_t lv_font_montserrat_18;
extern const lv_font_t lv_font_montserrat_20;
extern const lv_font_t lv_font_montserrat_22;
extern const lv_font_t lv_font_montserrat_14;
extern const lv_font_t lv_font_montserrat_32;
extern const lv_font_t lv_font_montserrat_36;
extern const lv_font_t lv_font_montserrat_48;
extern const lv_font_t lv_font_montserrat_12;

// Fallback aliases help static analysis when build_flags are not applied by the editor.
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

namespace
{

    TFT_eSPI g_tft;
    lv_disp_draw_buf_t g_draw_buf;
    lv_color_t *g_buf_a = nullptr;
    lv_color_t *g_buf_b = nullptr;
    lv_indev_t *g_touch_indev = nullptr;
    TouchDrvCSTXXX g_touch;
    uint8_t g_touch_addr = 0;

    bool g_lvgl_ready = false;
    bool g_touch_ready = false;

    lv_obj_t *g_boot_title = nullptr;
    lv_obj_t *g_boot_lcd = nullptr;
    lv_obj_t *g_boot_touch = nullptr;
    lv_obj_t *g_boot_sensor = nullptr;

    lv_obj_t *g_pages[3] = {nullptr, nullptr, nullptr};
    uint8_t g_current_page = 0;

    lv_obj_t *g_temp_value = nullptr;
    lv_obj_t *g_hum_value = nullptr;
    lv_obj_t *g_press_value = nullptr;
    lv_obj_t *g_alt_value = nullptr;
    lv_obj_t *g_footer_uptime = nullptr;

    lv_obj_t *g_iaq_arc = nullptr;
    lv_obj_t *g_iaq_value = nullptr;
    lv_obj_t *g_iaq_number = nullptr;
    lv_obj_t *g_iaq_status = nullptr;
    lv_obj_t *g_iaq_risk = nullptr;
    lv_obj_t *g_gas_value = nullptr;
    lv_obj_t *g_acc_value = nullptr;
    lv_obj_t *g_p2_temp = nullptr;
    lv_obj_t *g_p2_hum = nullptr;

    lv_obj_t *g_uptime_big = nullptr;
    lv_obj_t *g_cpu_value = nullptr;
    lv_obj_t *g_storage_value = nullptr;
    lv_obj_t *g_battery_labels[3] = {nullptr, nullptr, nullptr};
    lv_obj_t *g_env_headers[3] = {nullptr, nullptr, nullptr};

    uint32_t g_bat_filtered_mv = 0;
    bool g_bat_filtered_ready = false;
    int16_t g_bat_display_pct = -1;
    uint8_t g_bat_down_confirm = 0;
    uint8_t g_bat_up_confirm = 0;
    uint32_t g_last_bg_ui_refresh_ms = 0;
    bool g_touch_hw_awake = true;
    bool g_prev_display_awake = true;

    constexpr uint8_t BATTERY_ADC_AVG_SAMPLES = 8;
    constexpr uint8_t BATTERY_DOWN_CONFIRM_CYCLES = 2;
    constexpr uint8_t BATTERY_UP_CONFIRM_CYCLES = 4;
    constexpr uint8_t BATTERY_UP_MIN_STEP = 2;
    constexpr uint32_t UI_TASK_SLEEP_DELAY_MS = 120UL;
    constexpr uint32_t UI_BG_REFRESH_MS = 1000UL;
    constexpr uint32_t COLOR_BOOT_CHECKING = 0xFFD60A;

    uint32_t g_last_ui_refresh_ms = 0;
    uint32_t g_last_cpu_refresh_ms = 0;
    uint32_t g_last_sys_info_refresh_ms = 0;

    struct IaqDescriptor
    {
        const char *status;
        const char *risk;
        uint32_t color;
    };

    esp_adc_cal_characteristics_t g_bat_adc_chars;
    bool g_bat_adc_ready = false;

    int32_t clamp_i32(int32_t value, int32_t low, int32_t high)
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

    void format_uptime(char *out, size_t out_len, uint32_t uptime_seconds)
    {
        const uint32_t hours = uptime_seconds / 3600U;
        const uint32_t minutes = (uptime_seconds % 3600U) / 60U;
        const uint32_t seconds = uptime_seconds % 60U;
        snprintf(out, out_len, "%02lu:%02lu:%02lu", hours, minutes, seconds);
    }

    uint8_t battery_percent_from_mv(uint32_t mv)
    {
        if (mv <= BATTERY_MIN_MV)
        {
            return 0;
        }
        if (mv >= BATTERY_MAX_MV)
        {
            return 100;
        }
        return static_cast<uint8_t>(((mv - BATTERY_MIN_MV) * 100U) / (BATTERY_MAX_MV - BATTERY_MIN_MV));
    }

    lv_color_t blend_hex_colors(uint32_t from_hex, uint32_t to_hex, uint8_t mix_255)
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

    lv_color_t battery_color_from_percent(uint8_t percent, bool usb_power)
    {
        if (usb_power)
        {
            return lv_color_hex(0x2EDC72);
        }

        // Multi-stop ramp from green -> yellow -> orange -> red.
        static const uint8_t kStopsPercent[] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10};
        static const uint32_t kStopsColor[] = {
            0x2EDC72,
            0x4FD968,
            0x73D55D,
            0x97D153,
            0xBCD049,
            0xD9C444,
            0xEEAF3E,
            0xF99339,
            0xFD6D35,
            0xFF3B30,
        };

        const uint8_t p = static_cast<uint8_t>(clamp_i32(percent, 10, 100));
        const size_t stop_count = sizeof(kStopsPercent) / sizeof(kStopsPercent[0]);
        for (size_t i = 0; i + 1U < stop_count; ++i)
        {
            const uint8_t high = kStopsPercent[i];
            const uint8_t low = kStopsPercent[i + 1U];
            if ((p <= high) && (p >= low))
            {
                const uint8_t span = static_cast<uint8_t>(high - low);
                const uint8_t progress = static_cast<uint8_t>(high - p);
                const uint8_t mix_255 = (span == 0U) ? 0U : static_cast<uint8_t>((255U * progress) / span);
                return blend_hex_colors(kStopsColor[i], kStopsColor[i + 1U], mix_255);
            }
        }

        return lv_color_hex(kStopsColor[stop_count - 1U]);
    }

    uint8_t estimate_cpu_load_percent(uint32_t now_ms)
    {
        // This is a lightweight estimator (not true core utilization).
        // It combines heap usage baseline + memory churn + small time variation,
        // then smooths the result to keep UI readable.
        static uint32_t prev_heap_free = 0;
        static uint8_t smoothed = 0;

        const uint32_t heap_free = ESP.getFreeHeap();
        const uint32_t heap_total = ESP.getHeapSize();

        uint32_t heap_used_pct = 0;
        if (heap_total > 0U)
        {
            heap_used_pct = (100UL * (heap_total - heap_free)) / heap_total;
        }

        uint32_t heap_delta = 0;
        if (prev_heap_free != 0U)
        {
            heap_delta = (heap_free > prev_heap_free) ? (heap_free - prev_heap_free) : (prev_heap_free - heap_free);
        }
        prev_heap_free = heap_free;

        const uint32_t churn_term = (heap_delta > 2048U) ? 20U : (heap_delta / 100U);
        const uint32_t wave_term = (now_ms / 1000UL) % 7UL;

        const int32_t instant = clamp_i32(static_cast<int32_t>(8U + (heap_used_pct / 2U) + churn_term + wave_term), 5, 95);
        if (smoothed == 0U)
        {
            smoothed = static_cast<uint8_t>(instant);
        }
        else
        {
            smoothed = static_cast<uint8_t>((3U * smoothed + static_cast<uint8_t>(instant)) / 4U);
        }
        return smoothed;
    }

    IaqDescriptor describe_iaq(int32_t iaq)
    {
        if (iaq <= 50)
        {
            return {"Excellent", "Very Low", 0x39FF14};
        }
        if (iaq <= 100)
        {
            return {"Good", "Low", 0x6BDF3C};
        }
        if (iaq <= 150)
        {
            return {"Moderate", "Medium", 0xE8D33A};
        }
        if (iaq <= 200)
        {
            return {"Poor", "Elevated", 0xF4A13A};
        }
        if (iaq <= 300)
        {
            return {"Unhealthy", "High", 0xFF6C37};
        }
        return {"Hazardous", "Very High", 0xFF3B30};
    }

    const char *describe_bsec_accuracy(uint8_t accuracy)
    {
        switch (accuracy)
        {
        case 0:
            return "Very Low";
        case 1:
            return "Low";
        case 2:
            return "Medium";
        default:
            return "High";
        }
    }

    uint32_t read_battery_mv(bool *has_battery)
    {
        if (!g_bat_adc_ready)
        {
            analogReadResolution(12);
            analogSetPinAttenuation(PIN_BAT_VOLT, ADC_11db);
            esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &g_bat_adc_chars);
            g_bat_adc_ready = true;
        }

        uint32_t raw_sum = 0;
        for (uint8_t i = 0; i < BATTERY_ADC_AVG_SAMPLES; ++i)
        {
            raw_sum += static_cast<uint32_t>(analogRead(PIN_BAT_VOLT));
        }

        const uint32_t raw_avg = raw_sum / BATTERY_ADC_AVG_SAMPLES;
        const uint32_t mv = esp_adc_cal_raw_to_voltage(raw_avg, &g_bat_adc_chars) * 2U;

        if (!g_bat_filtered_ready)
        {
            g_bat_filtered_mv = mv;
            g_bat_filtered_ready = true;
        }
        else
        {
            g_bat_filtered_mv = (g_bat_filtered_mv * 3U + mv) / 4U;
        }

        const bool present = g_bat_filtered_mv <= BATTERY_ABSENT_MV;
        if (has_battery != nullptr)
        {
            *has_battery = present;
        }
        return g_bat_filtered_mv;
    }

    void create_battery_label(lv_obj_t *parent, uint8_t page_idx)
    {
        if (page_idx > 2)
        {
            return;
        }

        g_battery_labels[page_idx] = lv_label_create(parent);
        lv_label_set_text(g_battery_labels[page_idx], LV_SYMBOL_BATTERY_EMPTY " --%");
        lv_obj_set_style_text_color(g_battery_labels[page_idx], lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(g_battery_labels[page_idx], &lv_font_montserrat_12, 0);
        lv_obj_align(g_battery_labels[page_idx], LV_ALIGN_TOP_MID, 0, 8);
    }

    void update_battery_labels()
    {
        bool has_battery = false;
        const uint32_t bat_mv = read_battery_mv(&has_battery);
        const bool usb_power = !has_battery;
        const uint8_t bat_pct = battery_percent_from_mv(bat_mv);

        uint8_t shown_pct = 100U;
        if (usb_power)
        {
            g_bat_display_pct = -1;
            g_bat_down_confirm = 0;
            g_bat_up_confirm = 0;
        }
        else
        {
            if (g_bat_display_pct < 0)
            {
                g_bat_display_pct = static_cast<int16_t>(bat_pct);
            }
            else if (bat_pct < static_cast<uint8_t>(g_bat_display_pct))
            {
                ++g_bat_down_confirm;
                g_bat_up_confirm = 0;
                if (g_bat_down_confirm >= BATTERY_DOWN_CONFIRM_CYCLES)
                {
                    g_bat_display_pct = static_cast<int16_t>(bat_pct);
                    g_bat_down_confirm = 0;
                }
            }
            else if (bat_pct > static_cast<uint8_t>(g_bat_display_pct))
            {
                ++g_bat_up_confirm;
                g_bat_down_confirm = 0;
                if ((bat_pct >= static_cast<uint8_t>(g_bat_display_pct + BATTERY_UP_MIN_STEP)) &&
                    (g_bat_up_confirm >= BATTERY_UP_CONFIRM_CYCLES))
                {
                    g_bat_display_pct = static_cast<int16_t>(bat_pct);
                    g_bat_up_confirm = 0;
                }
            }
            else
            {
                g_bat_down_confirm = 0;
                g_bat_up_confirm = 0;
            }

            shown_pct = static_cast<uint8_t>(clamp_i32(g_bat_display_pct, 0, 100));
        }

        const lv_color_t text_color = battery_color_from_percent(shown_pct, usb_power);

        for (uint8_t i = 0; i < 3; ++i)
        {
            if (g_battery_labels[i] == nullptr)
            {
                continue;
            }

            lv_obj_set_style_text_color(g_battery_labels[i], text_color, 0);
            lv_obj_align(g_battery_labels[i], LV_ALIGN_TOP_MID, 0, 8);

            lv_label_set_text_fmt(g_battery_labels[i], LV_SYMBOL_BATTERY_FULL " %u%%", shown_pct);
        }
    }

    bool probe_touch_addr(uint8_t address)
    {
        Wire.beginTransmission(address);
        return Wire.endTransmission() == 0;
    }

    void sync_touch_power_state(bool display_awake)
    {
        if (!g_touch_ready)
        {
            return;
        }

        if (display_awake)
        {
            if (!g_touch_hw_awake)
            {
                g_touch.wakeup();
                delay(3);
                g_touch_hw_awake = true;
            }
        }
        else
        {
            if (g_touch_hw_awake)
            {
                g_touch.sleep();
                g_touch_hw_awake = false;
            }
        }
    }

    void touch_read_cb(lv_indev_drv_t * /*drv*/, lv_indev_data_t *data)
    {
        if (!power_is_display_awake() || !g_touch_hw_awake)
        {
            data->state = LV_INDEV_STATE_RELEASED;
            return;
        }

        if (g_touch_ready)
        {
            const TouchPoints &touch_points = g_touch.getTouchPoints();
            if (touch_points.hasPoints())
            {
                const TouchPoint &p = touch_points.getPoint(0);
                data->state = LV_INDEV_STATE_PRESSED;
                data->point.x = static_cast<lv_coord_t>(clamp_i32(p.x, 0, SCREEN_WIDTH - 1));
                data->point.y = static_cast<lv_coord_t>(clamp_i32(p.y, 0, SCREEN_HEIGHT - 1));
                wake_system_reset();
                return;
            }
        }

        data->state = LV_INDEV_STATE_RELEASED;
    }

    void display_flush_cb(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
    {
        const uint16_t width = static_cast<uint16_t>(area->x2 - area->x1 + 1);
        const uint16_t height = static_cast<uint16_t>(area->y2 - area->y1 + 1);

        g_tft.startWrite();
        g_tft.setAddrWindow(area->x1, area->y1, width, height);
        g_tft.pushColors(reinterpret_cast<uint16_t *>(&color_p->full), width * height, true);
        g_tft.endWrite();

        lv_disp_flush_ready(disp);
    }

    lv_obj_t *create_card(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                          const char *title, lv_obj_t **value_label)
    {
        lv_obj_t *card = lv_obj_create(parent);
        lv_obj_set_size(card, w, h);
        lv_obj_set_pos(card, x, y);
        lv_obj_set_style_bg_color(card, lv_color_hex(COLOR_CARD_BG), 0);
        lv_obj_set_style_border_color(card, lv_color_hex(COLOR_BORDER), 0);
        lv_obj_set_style_border_width(card, 1, 0);
        lv_obj_set_style_radius(card, 12, 0);
        lv_obj_set_style_pad_all(card, 8, 0);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *title_label = lv_label_create(card);
        lv_label_set_text(title_label, title);
        lv_obj_set_style_text_color(title_label, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 0, 0);

        *value_label = lv_label_create(card);
        lv_label_set_text(*value_label, "--");
        lv_obj_set_style_text_color(*value_label, lv_color_hex(COLOR_VALUE), 0);
        lv_obj_set_style_text_font(*value_label, &lv_font_montserrat_20, 0);
        lv_obj_align(*value_label, LV_ALIGN_BOTTOM_LEFT, 0, 2);

        return card;
    }

    void set_boot_line(lv_obj_t *label, const char *name, bool done, bool ok)
    {
        const uint32_t status_color = done ? (ok ? COLOR_STATUS : COLOR_FAIL) : COLOR_BOOT_CHECKING;
        const char *status_text = done ? (ok ? "OK" : "Fail") : "Checking";

        char line[96] = {0};
        snprintf(line,
                 sizeof(line),
                 "%s #%.6X [%s]#",
                 name,
                 status_color,
                 status_text);
        lv_label_set_recolor(label, true);
        lv_label_set_text(label, line);
    }

    void set_page(uint8_t next_page)
    {
        if (next_page > 2)
        {
            return;
        }

        g_current_page = next_page;
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (g_pages[i] == nullptr)
            {
                continue;
            }
            if (i == g_current_page)
            {
                lv_obj_clear_flag(g_pages[i], LV_OBJ_FLAG_HIDDEN);
            }
            else
            {
                lv_obj_add_flag(g_pages[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
    }

    void gesture_event_cb(lv_event_t *e)
    {
        if (lv_event_get_code(e) != LV_EVENT_GESTURE)
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
            set_page((g_current_page + 1U) % 3U);
        }
        else if (direction == LV_DIR_RIGHT)
        {
            set_page((g_current_page + 2U) % 3U);
        }
    }

    void build_page_one(lv_obj_t *parent)
    {
        g_env_headers[0] = lv_label_create(parent);
        lv_label_set_text(g_env_headers[0], LV_SYMBOL_DIRECTORY " Env_monitor");
        lv_obj_set_style_text_color(g_env_headers[0], lv_color_hex(COLOR_VALUE), 0);
        lv_obj_set_style_text_font(g_env_headers[0], &lv_font_montserrat_14, 0);
        lv_obj_align(g_env_headers[0], LV_ALIGN_TOP_LEFT, 8, 7);

        lv_obj_t *header_right = lv_label_create(parent);
        lv_label_set_text(header_right, "PAGE 01 / ENV");
        lv_obj_set_style_text_color(header_right, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(header_right, &lv_font_montserrat_12, 0);
        lv_obj_align(header_right, LV_ALIGN_TOP_RIGHT, -8, 8);

        create_battery_label(parent, 0);

        create_card(parent, 8, 30, 148, 58, "TEMP", &g_temp_value);
        create_card(parent, 164, 30, 148, 58, "HUMIDITY", &g_hum_value);
        create_card(parent, 8, 94, 148, 58, "PRESSURE", &g_press_value);
        create_card(parent, 164, 94, 148, 58, "ALTITUDE", &g_alt_value);

        g_footer_uptime = lv_label_create(parent);
        lv_label_set_text(g_footer_uptime, "Uptime: 00:00:00");
        lv_obj_set_style_text_color(g_footer_uptime, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(g_footer_uptime, &lv_font_montserrat_12, 0);
        lv_obj_align(g_footer_uptime, LV_ALIGN_BOTTOM_MID, 0, -4);
    }

    void build_page_two(lv_obj_t *parent)
    {
        g_env_headers[1] = lv_label_create(parent);
        lv_label_set_text(g_env_headers[1], LV_SYMBOL_DIRECTORY " Env_monitor");
        lv_obj_set_style_text_color(g_env_headers[1], lv_color_hex(COLOR_VALUE), 0);
        lv_obj_set_style_text_font(g_env_headers[1], &lv_font_montserrat_14, 0);
        lv_obj_align(g_env_headers[1], LV_ALIGN_TOP_LEFT, 8, 7);

        lv_obj_t *header_right = lv_label_create(parent);
        lv_label_set_text(header_right, "PAGE 02 / AQI");
        lv_obj_set_style_text_color(header_right, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(header_right, &lv_font_montserrat_12, 0);
        lv_obj_align(header_right, LV_ALIGN_TOP_RIGHT, -8, 8);

        create_battery_label(parent, 1);

        g_gas_value = lv_label_create(parent);
        lv_label_set_text(g_gas_value, "Gas res.\n--.- kOhm");
        lv_obj_set_style_text_color(g_gas_value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_font(g_gas_value, &lv_font_montserrat_14, 0);
        lv_obj_align(g_gas_value, LV_ALIGN_LEFT_MID, 8, -24);

        g_acc_value = lv_label_create(parent);
        lv_label_set_text(g_acc_value, "Accuracy\n-");
        lv_obj_set_style_text_color(g_acc_value, lv_color_hex(COLOR_STATUS), 0);
        lv_obj_set_style_text_font(g_acc_value, &lv_font_montserrat_14, 0);
        lv_obj_align(g_acc_value, LV_ALIGN_LEFT_MID, 8, 34);

        g_iaq_arc = lv_arc_create(parent);
        lv_obj_set_size(g_iaq_arc, 132, 132);
        lv_obj_align(g_iaq_arc, LV_ALIGN_CENTER, 0, 12);
        lv_arc_set_rotation(g_iaq_arc, 135);
        lv_arc_set_bg_angles(g_iaq_arc, 0, 270);
        lv_arc_set_range(g_iaq_arc, 0, 500);
        lv_arc_set_value(g_iaq_arc, 0);
        lv_obj_set_style_arc_width(g_iaq_arc, 10, LV_PART_MAIN);
        lv_obj_set_style_arc_width(g_iaq_arc, 10, LV_PART_INDICATOR);
        lv_obj_set_style_arc_color(g_iaq_arc, lv_color_hex(0x1C1C1C), LV_PART_MAIN);
        lv_obj_set_style_arc_color(g_iaq_arc, lv_color_hex(COLOR_STATUS), LV_PART_INDICATOR);
        lv_obj_remove_style(g_iaq_arc, nullptr, LV_PART_KNOB);
        lv_obj_clear_flag(g_iaq_arc, LV_OBJ_FLAG_CLICKABLE);

        g_iaq_value = lv_label_create(parent);
        lv_label_set_text(g_iaq_value, "IAQ");
        lv_obj_set_style_text_font(g_iaq_value, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(g_iaq_value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_width(g_iaq_value, 64);
        lv_obj_set_style_text_align(g_iaq_value, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align_to(g_iaq_value, g_iaq_arc, LV_ALIGN_CENTER, 0, -18);

        g_iaq_number = lv_label_create(parent);
        lv_label_set_text(g_iaq_number, "0");
        lv_obj_set_style_text_font(g_iaq_number, &lv_font_montserrat_32, 0);
        lv_obj_set_style_text_color(g_iaq_number, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_width(g_iaq_number, 96);
        lv_obj_set_style_text_align(g_iaq_number, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_align_to(g_iaq_number, g_iaq_arc, LV_ALIGN_CENTER, 0, 18);

        g_iaq_status = lv_label_create(parent);
        lv_label_set_text(g_iaq_status, "Status\n-");
        lv_obj_set_style_text_font(g_iaq_status, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(g_iaq_status, lv_color_hex(COLOR_STATUS), 0);
        lv_obj_align(g_iaq_status, LV_ALIGN_RIGHT_MID, -8, -24);

        g_iaq_risk = lv_label_create(parent);
        lv_label_set_text(g_iaq_risk, "Risk\n-");
        lv_obj_set_style_text_font(g_iaq_risk, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(g_iaq_risk, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(g_iaq_risk, LV_ALIGN_RIGHT_MID, -8, 34);

        g_p2_temp = lv_label_create(parent);
        lv_label_set_text(g_p2_temp, "--.- °C");
        lv_obj_set_style_text_color(g_p2_temp, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_align(g_p2_temp, LV_ALIGN_BOTTOM_LEFT, 8, -4);

        g_p2_hum = lv_label_create(parent);
        lv_label_set_text(g_p2_hum, "--.- %");
        lv_obj_set_style_text_color(g_p2_hum, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_align(g_p2_hum, LV_ALIGN_BOTTOM_RIGHT, -8, -4);
    }

    void build_page_three(lv_obj_t *parent)
    {
        g_env_headers[2] = lv_label_create(parent);
        lv_label_set_text(g_env_headers[2], LV_SYMBOL_DIRECTORY " Env_monitor");
        lv_obj_set_style_text_color(g_env_headers[2], lv_color_hex(COLOR_VALUE), 0);
        lv_obj_set_style_text_font(g_env_headers[2], &lv_font_montserrat_14, 0);
        lv_obj_align(g_env_headers[2], LV_ALIGN_TOP_LEFT, 8, 7);

        lv_obj_t *header_right = lv_label_create(parent);
        lv_label_set_text(header_right, "PAGE 03 / Uptime");
        lv_obj_set_style_text_color(header_right, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(header_right, &lv_font_montserrat_12, 0);
        lv_obj_align(header_right, LV_ALIGN_TOP_RIGHT, -8, 8);

        create_battery_label(parent, 2);

        g_uptime_big = lv_label_create(parent);
        lv_label_set_text(g_uptime_big, "00:00:00");
        lv_obj_set_style_text_color(g_uptime_big, lv_color_hex(0xCCFFFF), 0);
        lv_obj_set_style_text_font(g_uptime_big, &lv_font_montserrat_36, 0);
        lv_obj_align(g_uptime_big, LV_ALIGN_TOP_MID, 0, 34);

        lv_obj_t *cpu_card = lv_obj_create(parent);
        lv_obj_set_size(cpu_card, 148, 64);
        lv_obj_set_pos(cpu_card, 8, 98);
        lv_obj_set_style_bg_color(cpu_card, lv_color_hex(COLOR_CARD_BG), 0);
        lv_obj_set_style_border_color(cpu_card, lv_color_hex(COLOR_BORDER), 0);
        lv_obj_set_style_radius(cpu_card, 12, 0);
        lv_obj_clear_flag(cpu_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *cpu_label = lv_label_create(cpu_card);
        lv_label_set_text(cpu_label, LV_SYMBOL_SETTINGS " CPU Load");
        lv_obj_set_style_text_color(cpu_label, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(cpu_label, &lv_font_montserrat_12, 0);
        lv_obj_align(cpu_label, LV_ALIGN_TOP_LEFT, 0, 0);

        g_cpu_value = lv_label_create(cpu_card);
        lv_label_set_text(g_cpu_value, "0%");
        lv_obj_set_style_text_font(g_cpu_value, &lv_font_montserrat_22, 0);
        lv_obj_set_style_text_color(g_cpu_value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(g_cpu_value, LV_ALIGN_BOTTOM_LEFT, 0, 2);

        lv_obj_t *storage_card = lv_obj_create(parent);
        lv_obj_set_size(storage_card, 148, 64);
        lv_obj_set_pos(storage_card, 164, 98);
        lv_obj_set_style_bg_color(storage_card, lv_color_hex(COLOR_CARD_BG), 0);
        lv_obj_set_style_border_color(storage_card, lv_color_hex(COLOR_BORDER), 0);
        lv_obj_set_style_radius(storage_card, 12, 0);
        lv_obj_clear_flag(storage_card, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *storage_label = lv_label_create(storage_card);
        lv_label_set_text(storage_label, LV_SYMBOL_DRIVE " Storage");
        lv_obj_set_style_text_color(storage_label, lv_color_hex(COLOR_TEXT_DIM), 0);
        lv_obj_set_style_text_font(storage_label, &lv_font_montserrat_12, 0);
        lv_obj_align(storage_label, LV_ALIGN_TOP_LEFT, 0, 0);

        g_storage_value = lv_label_create(storage_card);
        lv_label_set_text(g_storage_value, "--/-- MB");
        lv_obj_set_style_text_font(g_storage_value, &lv_font_montserrat_18, 0);
        lv_obj_set_style_text_color(g_storage_value, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(g_storage_value, LV_ALIGN_BOTTOM_LEFT, 0, 2);
    }

    void update_page_values()
    {
        const uint32_t now_ms = millis();
        if (now_ms - g_last_ui_refresh_ms < 500)
        {
            return;
        }
        g_last_ui_refresh_ms = now_ms;

        const SensorData data = sensors_get_data();
        char text[64] = {0};

        const bool sensor_connected_now = sensors_is_connected_realtime();
        const lv_color_t env_color = sensor_connected_now ? lv_color_hex(COLOR_VALUE) : lv_color_hex(COLOR_FAIL);
        for (uint8_t i = 0; i < 3; ++i)
        {
            if (g_env_headers[i] != nullptr)
            {
                lv_obj_set_style_text_color(g_env_headers[i], env_color, 0);
            }
        }

        if (data.valid)
        {
            snprintf(text, sizeof(text), "%.2f °C", data.temperature_c);
            lv_label_set_text(g_temp_value, text);

            snprintf(text, sizeof(text), "%.2f %%", data.humidity_pct);
            lv_label_set_text(g_hum_value, text);

            snprintf(text, sizeof(text), "%.2f hPa", data.pressure_hpa);
            lv_label_set_text(g_press_value, text);

            snprintf(text, sizeof(text), "%.1f m", data.altitude_m);
            lv_label_set_text(g_alt_value, text);

            snprintf(text, sizeof(text), "%.2f kOhm", data.gas_resistance_kohm);
            lv_label_set_text_fmt(g_gas_value, "Gas res.\n%s", text);

            const uint8_t acc_level = data.iaq_accuracy > 3U ? 3U : data.iaq_accuracy;
            lv_label_set_text_fmt(g_acc_value,
                                  "Accuracy\n%s (%u)",
                                  describe_bsec_accuracy(acc_level),
                                  acc_level);

            float iaq_display = data.iaq;
            if (!isfinite(iaq_display) && isfinite(data.iaq_static))
            {
                iaq_display = data.iaq_static;
            }

            int32_t iaq_value = 0;
            if (isfinite(iaq_display))
            {
                const int32_t iaq_tenths = static_cast<int32_t>(lroundf(iaq_display * 10.0f));
                const int32_t iaq_whole = iaq_tenths / 10;
                int32_t iaq_frac = iaq_tenths % 10;
                if (iaq_frac < 0)
                {
                    iaq_frac = -iaq_frac;
                }
                char iaq_text[16] = {0};
                snprintf(iaq_text, sizeof(iaq_text), "%ld.%ld", static_cast<long>(iaq_whole), static_cast<long>(iaq_frac));
                lv_label_set_text(g_iaq_number, iaq_text);
                iaq_value = static_cast<int32_t>(lroundf(iaq_display));
            }
            else
            {
                lv_label_set_text(g_iaq_number, "--");
            }

            iaq_value = clamp_i32(iaq_value, 0, 500);
            lv_arc_set_value(g_iaq_arc, iaq_value);

            const IaqDescriptor iaq_desc = describe_iaq(iaq_value);
            lv_label_set_text_fmt(g_iaq_status, "Status\n%s", iaq_desc.status);
            lv_label_set_text_fmt(g_iaq_risk, "Risk\n%s", iaq_desc.risk);
            lv_obj_set_style_arc_color(g_iaq_arc, lv_color_hex(iaq_desc.color), LV_PART_INDICATOR);
            lv_obj_set_style_text_color(g_iaq_status, lv_color_hex(iaq_desc.color), 0);
            lv_obj_set_style_text_color(g_iaq_risk, lv_color_hex(0xFFFFFF), 0);

            snprintf(text, sizeof(text), "%.2f °C", data.temperature_c);
            lv_label_set_text(g_p2_temp, text);
            snprintf(text, sizeof(text), "%.2f %%", data.humidity_pct);
            lv_label_set_text(g_p2_hum, text);
        }

        char uptime[16] = {0};
        const uint64_t uptime_seconds = static_cast<uint64_t>(esp_timer_get_time()) / 1000000ULL;
        format_uptime(uptime, sizeof(uptime), static_cast<uint32_t>(uptime_seconds));
        lv_label_set_text_fmt(g_footer_uptime, "Uptime: %s", uptime);
        lv_label_set_text(g_uptime_big, uptime);

        if ((g_last_cpu_refresh_ms == 0U) || (now_ms - g_last_cpu_refresh_ms >= CPU_LOAD_REFRESH_MS))
        {
            g_last_cpu_refresh_ms = now_ms;
            const uint8_t cpu_est = estimate_cpu_load_percent(now_ms);
            lv_label_set_text_fmt(g_cpu_value, "%u %%", cpu_est);
        }

        if ((g_last_sys_info_refresh_ms == 0U) || (now_ms - g_last_sys_info_refresh_ms >= SYS_INFO_REFRESH_MS))
        {
            g_last_sys_info_refresh_ms = now_ms;
            const uint32_t storage_total = ESP.getFlashChipSize();
            const uint32_t storage_free = ESP.getFreeSketchSpace();
            const uint32_t bytes_per_mb = 1024UL * 1024UL;

            uint32_t storage_free_tenths = 0;
            uint32_t storage_total_tenths = 0;
            if (storage_total > 0U)
            {
                storage_free_tenths = static_cast<uint32_t>((static_cast<uint64_t>(storage_free) * 10ULL) / bytes_per_mb);
                storage_total_tenths = static_cast<uint32_t>((static_cast<uint64_t>(storage_total) * 10ULL) / bytes_per_mb);
            }

            lv_label_set_text_fmt(g_storage_value,
                                  "%lu.%lu/%lu.%lu MB",
                                  static_cast<unsigned long>(storage_free_tenths / 10U),
                                  static_cast<unsigned long>(storage_free_tenths % 10U),
                                  static_cast<unsigned long>(storage_total_tenths / 10U),
                                  static_cast<unsigned long>(storage_total_tenths % 10U));

            update_battery_labels();
        }
    }

} // namespace

bool ui_init_display()
{
    pinMode(PIN_LCD_POWER, OUTPUT);
    digitalWrite(PIN_LCD_POWER, HIGH);
    pinMode(PIN_LCD_BL, OUTPUT);
    digitalWrite(PIN_LCD_BL, HIGH);
    pinMode(PIN_BAT_VOLT, INPUT);

    lv_init();

    g_tft.begin();
    g_tft.setRotation(LCD_ROTATION);
    g_tft.fillScreen(TFT_BLACK);

    const size_t buf_pixels = SCREEN_WIDTH * 20;
    g_buf_a = static_cast<lv_color_t *>(heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    g_buf_b = static_cast<lv_color_t *>(heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_DMA));
    if (g_buf_a == nullptr || g_buf_b == nullptr)
    {
        return false;
    }

    lv_disp_draw_buf_init(&g_draw_buf, g_buf_a, g_buf_b, buf_pixels);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = SCREEN_WIDTH;
    disp_drv.ver_res = SCREEN_HEIGHT;
    disp_drv.flush_cb = display_flush_cb;
    disp_drv.draw_buf = &g_draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read_cb;
    g_touch_indev = lv_indev_drv_register(&indev_drv);

    g_lvgl_ready = true;
    return true;
}

bool ui_init_touch()
{
    pinMode(PIN_TOUCH_RST, OUTPUT);
    pinMode(PIN_TOUCH_INT, INPUT_PULLUP);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 400000);

    digitalWrite(PIN_TOUCH_RST, LOW);
    delay(30);
    digitalWrite(PIN_TOUCH_RST, HIGH);
    delay(80);

    g_touch_addr = 0;
    if (probe_touch_addr(0x15))
    {
        g_touch_addr = 0x15;
    }
    else if (probe_touch_addr(0x1A))
    {
        g_touch_addr = 0x1A;
    }

    if (g_touch_addr == 0)
    {
        g_touch_ready = false;
        return false;
    }

    g_touch.setPins(PIN_TOUCH_RST, PIN_TOUCH_INT);
    g_touch_ready = g_touch.begin(Wire, g_touch_addr, PIN_I2C_SDA, PIN_I2C_SCL);

    if (!g_touch_ready)
    {
        return false;
    }

    g_touch.setMaxCoordinates(SCREEN_WIDTH, SCREEN_HEIGHT);
    g_touch.setMirrorXY(true, false);
    g_touch.setSwapXY(true);
    g_touch.disableAutoSleep();
    g_touch.setCenterButtonCoordinate(85, 360);

    g_touch_hw_awake = true;
    g_prev_display_awake = power_is_display_awake();
    sync_touch_power_state(g_prev_display_awake);

    wake_system_reset();
    return true;
}

void ui_boot_diag_begin()
{
    if (!g_lvgl_ready)
    {
        return;
    }

    lv_obj_t *screen = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(screen, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_border_width(screen, 0, 0);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

    g_boot_title = lv_label_create(screen);
    lv_label_set_text(g_boot_title, "SYSTEM SELF-CHECK");
    lv_obj_set_style_text_color(g_boot_title, lv_color_hex(COLOR_VALUE), 0);
    lv_obj_set_style_text_font(g_boot_title, &lv_font_montserrat_22, 0);
    lv_obj_align(g_boot_title, LV_ALIGN_TOP_MID, 0, 12);

    g_boot_lcd = lv_label_create(screen);
    g_boot_touch = lv_label_create(screen);
    g_boot_sensor = lv_label_create(screen);

    lv_obj_set_style_text_font(g_boot_lcd, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_font(g_boot_touch, &lv_font_montserrat_18, 0);
    lv_obj_set_style_text_font(g_boot_sensor, &lv_font_montserrat_18, 0);

    lv_obj_align(g_boot_lcd, LV_ALIGN_TOP_LEFT, 16, 56);
    lv_obj_align(g_boot_touch, LV_ALIGN_TOP_LEFT, 16, 84);
    lv_obj_align(g_boot_sensor, LV_ALIGN_TOP_LEFT, 16, 112);

    lv_scr_load(screen);
}

void ui_boot_diag_update(const BootDiagStatus &status)
{
    if (!g_lvgl_ready || g_boot_lcd == nullptr)
    {
        return;
    }

    set_boot_line(g_boot_lcd, "LCD Initializing...", status.lcd_done, status.lcd_ok);
    set_boot_line(g_boot_touch, "Touch Controller...", status.touch_done, status.touch_ok);
    set_boot_line(g_boot_sensor, "BME680 Sensor...", status.sensor_done, status.sensor_ok);

    lv_tick_inc(20);
    lv_timer_handler();
    delay(20);
}

void ui_boot_diag_finish(uint32_t hold_ms)
{
    if (!g_lvgl_ready)
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

void ui_build_pages()
{
    if (!g_lvgl_ready)
    {
        return;
    }

    lv_obj_t *root = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(root, lv_color_hex(COLOR_BG), 0);
    lv_obj_set_style_border_width(root, 0, 0);
    lv_obj_clear_flag(root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(root, gesture_event_cb, LV_EVENT_GESTURE, nullptr);

    for (uint8_t i = 0; i < 3; ++i)
    {
        g_pages[i] = lv_obj_create(root);
        lv_obj_set_size(g_pages[i], SCREEN_WIDTH, SCREEN_HEIGHT);
        lv_obj_set_pos(g_pages[i], 0, 0);
        lv_obj_set_style_bg_opa(g_pages[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(g_pages[i], 0, 0);
        lv_obj_set_style_pad_all(g_pages[i], 0, 0);
        lv_obj_clear_flag(g_pages[i], LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_clear_flag(g_pages[i], LV_OBJ_FLAG_CLICKABLE);
    }

    build_page_one(g_pages[0]);
    build_page_two(g_pages[1]);
    build_page_three(g_pages[2]);
    set_page(0);

    lv_scr_load(root);
}

void uiTask(void * /*parameter*/)
{
    for (;;)
    {
        if (g_lvgl_ready)
        {
            const bool display_awake = power_is_display_awake();
            const uint32_t task_delay_ms = display_awake ? UI_TASK_DELAY_MS : UI_TASK_SLEEP_DELAY_MS;

            if (display_awake != g_prev_display_awake)
            {
                sync_touch_power_state(display_awake);
                g_prev_display_awake = display_awake;
            }

            lv_tick_inc(task_delay_ms);
            lv_timer_handler();

            if (display_awake)
            {
                update_page_values();
            }
            else
            {
                const uint32_t now_ms = millis();
                if ((g_last_bg_ui_refresh_ms == 0U) || (now_ms - g_last_bg_ui_refresh_ms >= UI_BG_REFRESH_MS))
                {
                    g_last_bg_ui_refresh_ms = now_ms;
                    update_page_values();
                }
            }

            vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(UI_TASK_DELAY_MS));
    }
}