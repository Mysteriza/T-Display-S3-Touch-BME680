#pragma once

#include <Arduino.h>

class SensorManager;

class SerialCLI
{
public:
    explicit SerialCLI(SensorManager &sensor_manager);
    void tick();

private:
    void trimInPlace(char *text) const;
    void toLowerAscii(char *text) const;
    void handleCommand(char *line);

private:
    SensorManager &sensor_manager_;
    char line_buffer_[96] = {0};
    size_t line_len_ = 0;
    uint32_t last_input_ms_ = 0;
};
