#pragma once

#include <Arduino.h>

struct BootDiagStatus
{
    bool lcd_done;
    bool lcd_ok;
    bool touch_done;
    bool touch_ok;
    bool sensor_done;
    bool sensor_ok;
};

bool ui_init_display();
bool ui_init_touch();
void ui_boot_diag_begin();
void ui_boot_diag_update(const BootDiagStatus &status);
void ui_boot_diag_finish(uint32_t hold_ms);
void ui_build_pages();
void uiTask(void *parameter);