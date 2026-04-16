#include "power_mgmt.h"

#include <driver/gpio.h>
#include <esp32-hal-cpu.h>

#include "config.h"

PowerManager *PowerManager::self_ = nullptr;

PowerManager &PowerManager::instance()
{
    static PowerManager manager;
    return manager;
}

uint32_t PowerManager::isrMillis() const
{
    return static_cast<uint32_t>(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

void PowerManager::applyCpuProfile(bool display_awake)
{
    const uint32_t target_mhz = display_awake ? cfg::power::kCpuFreqActiveMhz : cfg::power::kCpuFreqSleepMhz;
    const uint32_t current_mhz = static_cast<uint32_t>(getCpuFrequencyMhz());
    if (current_mhz != target_mhz)
    {
        setCpuFrequencyMhz(target_mhz);
    }
}

void IRAM_ATTR PowerManager::wakeFromInteractionISR()
{
    const uint32_t now_ms = xPortInIsrContext() ? isrMillis() : millis();
    last_interaction_ms_ = now_ms;
    display_awake_ = true;
    gpio_set_level(static_cast<gpio_num_t>(cfg::pins::kLcdBacklight), 1);
}

void PowerManager::wakeFromInteraction()
{
    wakeFromInteractionISR();
}

void IRAM_ATTR PowerManager::handleButtonWakeIsr()
{
    const uint32_t now_ms = isrMillis();
    if (now_ms - last_button_isr_ms_ < cfg::timing::kButtonDebounceMs)
    {
        return;
    }

    if (gpio_get_level(static_cast<gpio_num_t>(cfg::pins::kButtonWake)) != 0)
    {
        return;
    }

    last_button_isr_ms_ = now_ms;
    wakeFromInteractionISR();
}

void IRAM_ATTR PowerManager::buttonWakeIsrThunk()
{
    if (self_ != nullptr)
    {
        self_->handleButtonWakeIsr();
    }
}

void PowerManager::init()
{
    self_ = this;

    pinMode(cfg::pins::kLcdBacklight, OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(cfg::pins::kLcdBacklight), 1);

    pinMode(cfg::pins::kButtonWake, INPUT_PULLUP);

    last_interaction_ms_ = millis();
    display_awake_ = true;
    last_profile_awake_ = true;

    applyCpuProfile(true);

    attachInterrupt(digitalPinToInterrupt(cfg::pins::kButtonWake), PowerManager::buttonWakeIsrThunk, FALLING);
}

void PowerManager::loop()
{
    const uint32_t now_ms = millis();
    if (display_awake_ && (now_ms - last_interaction_ms_ > cfg::timing::kDisplayTimeoutMs))
    {
        gpio_set_level(static_cast<gpio_num_t>(cfg::pins::kLcdBacklight), 0);
        display_awake_ = false;
    }

    if (display_awake_ != last_profile_awake_)
    {
        applyCpuProfile(display_awake_);
        last_profile_awake_ = display_awake_;
    }
}

bool PowerManager::isDisplayAwake() const
{
    return display_awake_;
}

uint32_t PowerManager::lastInteractionMs() const
{
    return last_interaction_ms_;
}