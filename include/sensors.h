#pragma once

#include <Arduino.h>

struct SensorData
{
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
    float altitude_m;
    float gas_resistance_kohm;
    float iaq;
    uint8_t iaq_accuracy;
    bool valid;
    uint32_t last_update_ms;
};

bool sensors_init();
bool sensors_is_healthy();
SensorData sensors_get_data();
void sensors_debug_tick();
void sensorTask(void *parameter);