#include "power_mgmt.h"

#include <driver/gpio.h>
#include "config.h"

static volatile uint32_t g_last_interaction_ms = 0;
static volatile uint32_t g_last_button_isr_ms = 0;
static volatile bool g_display_awake = true;

static inline uint32_t isr_millis()
{
    return static_cast<uint32_t>(xTaskGetTickCountFromISR() * portTICK_PERIOD_MS);
}

void IRAM_ATTR wake_system_reset()
{
    const uint32_t now_ms = xPortInIsrContext() ? isr_millis() : millis();
    g_last_interaction_ms = now_ms;
    g_display_awake = true;
    gpio_set_level(static_cast<gpio_num_t>(PIN_LCD_BL), 1);
}

static void IRAM_ATTR button_wake_isr()
{
    const uint32_t now_ms = isr_millis();
    if (now_ms - g_last_button_isr_ms < BUTTON_DEBOUNCE_MS)
    {
        return;
    }

    g_last_button_isr_ms = now_ms;
    wake_system_reset();
}

void power_mgmt_init()
{
    pinMode(PIN_LCD_BL, OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(PIN_LCD_BL), 1);

    pinMode(PIN_BUTTON_WAKE, INPUT_PULLUP);
    g_last_interaction_ms = millis();
    g_display_awake = true;

    // CHANGE covers both rising/falling edges as requested.
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_WAKE), button_wake_isr, CHANGE);
}

void power_mgmt_loop()
{
    const uint32_t now_ms = millis();
    if (g_display_awake && (now_ms - g_last_interaction_ms > DISPLAY_TIMEOUT))
    {
        gpio_set_level(static_cast<gpio_num_t>(PIN_LCD_BL), 0);
        g_display_awake = false;
    }
}

uint32_t power_get_last_interaction_time()
{
    return g_last_interaction_ms;
}

bool power_is_display_awake()
{
    return g_display_awake;
}