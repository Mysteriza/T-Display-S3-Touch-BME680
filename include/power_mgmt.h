#pragma once

#include <Arduino.h>

void power_mgmt_init();
void power_mgmt_loop();
void IRAM_ATTR wake_system_reset();
uint32_t power_get_last_interaction_time();
bool power_is_display_awake();