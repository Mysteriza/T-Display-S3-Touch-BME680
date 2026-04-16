#pragma once

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <TouchDrv.hpp>
#include <esp_adc_cal.h>
#include <lvgl.h>

struct SensorData;

struct BootDiagStatus
{
    bool lcd_done = false;
    bool lcd_ok = false;
    bool touch_done = false;
    bool touch_ok = false;
    bool sensor_done = false;
    bool sensor_ok = false;
};

class UiController
{
public:
    static UiController &instance();

    bool initDisplay();
    bool initTouch();

    void bootDiagBegin();
    void bootDiagUpdate(const BootDiagStatus &status);
    void bootDiagFinish(uint32_t hold_ms);

    void buildPages();

    void taskLoop();
    static void taskEntry(void *parameter);

private:
    UiController() = default;
    UiController(const UiController &) = delete;
    UiController &operator=(const UiController &) = delete;

    struct PageEnvData
    {
        lv_obj_t *temp = nullptr;
        lv_obj_t *humidity = nullptr;
        lv_obj_t *pressure = nullptr;
        lv_obj_t *altitude = nullptr;
        lv_obj_t *uptime_footer = nullptr;
    };

    struct PageAqiData
    {
        lv_obj_t *gas = nullptr;
        lv_obj_t *accuracy = nullptr;
        lv_obj_t *iaq_arc = nullptr;
        lv_obj_t *iaq_number = nullptr;
        lv_obj_t *iaq_status = nullptr;
        lv_obj_t *temp = nullptr;
        lv_obj_t *humidity = nullptr;
    };

    struct PageSysData
    {
        lv_obj_t *uptime = nullptr;
        lv_obj_t *cpu_load = nullptr;
        lv_obj_t *storage = nullptr;
    };

    struct IaqDescriptor
    {
        const char *status;
        uint32_t status_color;
    };

    static void displayFlushCallback(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
    static void touchReadCallback(lv_indev_drv_t *drv, lv_indev_data_t *data);
    static void gestureEventCallback(lv_event_t *event);

    void onDisplayFlush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
    void onTouchRead(lv_indev_data_t *data);
    void onGesture(lv_event_t *event);

    void buildPageEnv(lv_obj_t *parent);
    void buildPageAqi(lv_obj_t *parent);
    void buildPageSys(lv_obj_t *parent);

    lv_obj_t *createHeader(lv_obj_t *parent, const char *page_info, uint8_t page_index);
    lv_obj_t *createValueCard(lv_obj_t *parent, lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                              const char *title, lv_obj_t **value_label, const lv_font_t *value_font);

    void setPage(uint8_t page_index);
    void updateValues();

    void syncTouchPowerState(bool display_awake);
    bool probeTouchAddress(uint8_t address);

    void updateBatteryLabels();
    uint32_t readBatteryMv(bool *has_battery);
    uint8_t batteryPercentFromMv(uint32_t mv) const;
    lv_color_t blendHexColors(uint32_t from_hex, uint32_t to_hex, uint8_t mix_255) const;
    lv_color_t batteryColorFromPercent(uint8_t percent, bool usb_power) const;

    IaqDescriptor describeIaqStatus(int32_t iaq_value) const;

    void formatUptime(char *out, size_t out_len, uint32_t uptime_seconds) const;
    int32_t clampI32(int32_t value, int32_t low, int32_t high) const;
    int32_t roundToInt(float value) const;

    void setBootLine(lv_obj_t *label, const char *name, bool done, bool ok);

private:
    TFT_eSPI tft_;
    TouchDrvCSTXXX touch_;

    lv_disp_draw_buf_t draw_buf_{};
    lv_color_t *buf_a_ = nullptr;
    lv_color_t *buf_b_ = nullptr;

    lv_indev_t *touch_indev_ = nullptr;

    bool lvgl_ready_ = false;
    bool touch_ready_ = false;
    bool touch_hw_awake_ = true;
    bool prev_display_awake_ = true;

    uint8_t touch_addr_ = 0;

    lv_obj_t *boot_title_ = nullptr;
    lv_obj_t *boot_lcd_ = nullptr;
    lv_obj_t *boot_touch_ = nullptr;
    lv_obj_t *boot_sensor_ = nullptr;

    lv_obj_t *pages_[3] = {nullptr, nullptr, nullptr};
    lv_obj_t *env_headers_[3] = {nullptr, nullptr, nullptr};
    lv_obj_t *battery_labels_[3] = {nullptr, nullptr, nullptr};

    uint8_t current_page_ = 0;

    PageEnvData page_env_{};
    PageAqiData page_aqi_{};
    PageSysData page_sys_{};

    esp_adc_cal_characteristics_t battery_adc_chars_{};
    bool battery_adc_ready_ = false;

    uint32_t bat_filtered_mv_ = 0;
    bool bat_filtered_ready_ = false;
    int16_t bat_display_pct_ = -1;
    uint8_t bat_down_confirm_ = 0;
    uint8_t bat_up_confirm_ = 0;

    uint32_t last_ui_refresh_ms_ = 0;
    uint32_t last_cpu_refresh_ms_ = 0;
    uint32_t last_sys_info_refresh_ms_ = 0;
    uint32_t last_bg_ui_refresh_ms_ = 0;
};
