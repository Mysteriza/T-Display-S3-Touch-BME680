#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "power_mgmt.h"
#include "sensor_manager.h"
#include "ui_controller.h"

namespace
{
  struct BootDataCheckResult
  {
    bool data_ok = false;
    bool iaq_ok = false;
  };

  BootDataCheckResult verifyDataAndIaq(SensorManager &sensor_manager)
  {
    BootDataCheckResult result;
    const uint32_t start_ms = millis();

    while (millis() - start_ms < cfg::timing::kBootDataCheckMs)
    {
      const SensorData snapshot = sensor_manager.getData();
      const bool core_valid = snapshot.valid &&
                              isfinite(snapshot.temperature_c) &&
                              isfinite(snapshot.humidity_pct) &&
                              isfinite(snapshot.pressure_hpa) &&
                              (snapshot.last_update_ms != 0U);

      float iaq_value = snapshot.iaq;
      if (!isfinite(iaq_value) && isfinite(snapshot.iaq_static))
      {
        iaq_value = snapshot.iaq_static;
      }

      const bool iaq_valid = isfinite(iaq_value) && (iaq_value >= 0.0f) && (iaq_value <= 500.0f);
      const bool accuracy_valid = snapshot.iaq_accuracy <= 3U;

      result.data_ok = core_valid;
      result.iaq_ok = iaq_valid && accuracy_valid;

      if (result.data_ok && result.iaq_ok)
      {
        break;
      }

      delay(60);
    }

    return result;
  }
}

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

  if (boot_status.sensor_ok)
  {
    const BootDataCheckResult check = verifyDataAndIaq(sensor_manager);
    boot_status.data_done = true;
    boot_status.data_ok = check.data_ok;
    boot_status.iaq_done = true;
    boot_status.iaq_ok = check.iaq_ok;
    ui.bootDiagUpdate(boot_status);
  }
  else
  {
    boot_status.data_done = true;
    boot_status.data_ok = false;
    boot_status.iaq_done = true;
    boot_status.iaq_ok = false;
    ui.bootDiagUpdate(boot_status);
  }

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