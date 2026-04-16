#include <Arduino.h>

#include "power_mgmt.h"
#include "sensor_manager.h"
#include "ui_controller.h"

void setup()
{
  Serial.begin(115200);
  delay(120);

  PowerManager::instance().init();

  UiController &ui = UiController::instance();
  SensorManager &sensor_manager = SensorManager::instance();

  BootDiagStatus boot_status{};

  boot_status.lcd_done = true;
  boot_status.lcd_ok = ui.initDisplay();
  ui.bootDiagBegin();
  ui.bootDiagUpdate(boot_status);

  boot_status.touch_done = true;
  boot_status.touch_ok = ui.initTouch();
  ui.bootDiagUpdate(boot_status);

  boot_status.sensor_done = true;
  boot_status.sensor_ok = sensor_manager.init();
  ui.bootDiagUpdate(boot_status);
  ui.bootDiagFinish(3000);

  ui.buildPages();

  xTaskCreatePinnedToCore(SensorManager::taskEntry, "sensorTask", 8192, &sensor_manager, 2, nullptr, 0);
  xTaskCreatePinnedToCore(UiController::taskEntry, "uiTask", 12288, &ui, 2, nullptr, 1);
}

void loop()
{
  static SerialCLI serial_cli(SensorManager::instance());
  serial_cli.tick();
  SensorManager::instance().debugTick();
  PowerManager::instance().loop();
  delay(20);
}