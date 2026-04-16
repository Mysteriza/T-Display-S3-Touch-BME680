#pragma once

#include <Arduino.h>

class PowerManager
{
public:
    static PowerManager &instance();

    void init();
    void loop();

    void wakeFromInteraction();
    void IRAM_ATTR wakeFromInteractionISR();

    bool isDisplayAwake() const;
    uint32_t lastInteractionMs() const;

private:
    PowerManager() = default;
    PowerManager(const PowerManager &) = delete;
    PowerManager &operator=(const PowerManager &) = delete;

    static void IRAM_ATTR buttonWakeIsrThunk();
    void IRAM_ATTR handleButtonWakeIsr();

    uint32_t isrMillis() const;
    void applyCpuProfile(bool display_awake);

private:
    static PowerManager *self_;

    volatile uint32_t last_interaction_ms_ = 0;
    volatile uint32_t last_button_isr_ms_ = 0;
    volatile bool display_awake_ = true;
    bool last_profile_awake_ = true;
};