#include <Arduino.h>
#include "power_mgmt.h"
#include "sensors.h"
#include "ui_pages.h"

void setup()
{
  Serial.begin(115200);
  delay(120);

  power_mgmt_init();

  BootDiagStatus boot_status{};

  boot_status.lcd_ok = ui_init_display();
  ui_boot_diag_begin();
  ui_boot_diag_update(boot_status);

  boot_status.touch_ok = ui_init_touch();
  ui_boot_diag_update(boot_status);

  boot_status.sensor_ok = sensors_init();
  ui_boot_diag_update(boot_status);
  ui_boot_diag_finish(3000);

  ui_build_pages();

  xTaskCreatePinnedToCore(sensorTask, "sensorTask", 8192, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(uiTask, "uiTask", 12288, nullptr, 2, nullptr, 1);
}

void loop()
{
  sensors_serial_tick();
  sensors_debug_tick();
  power_mgmt_loop();
  delay(20);
}