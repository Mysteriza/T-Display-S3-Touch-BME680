#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "power_mgmt.h"
#include "serial_cli.h"
#include "sensor_manager.h"
#include "ui_controller.h"

namespace
{
  struct BootDataCheckResult
  {
    bool data_ok = false;
  };

  BootDataCheckResult verifySensorData(SensorManager &sensor_manager)
  {
    BootDataCheckResult result;
    const uint32_t start_ms = millis();
    constexpr uint32_t kFreshDataMaxAgeMs = 4000U;

    while (millis() - start_ms < cfg::timing::kBootDataCheckMs)
    {
      const uint32_t now_ms = millis();
      const SensorData snapshot = sensor_manager.getData();
      const bool data_fresh = (snapshot.last_update_ms != 0U) && ((now_ms - snapshot.last_update_ms) <= kFreshDataMaxAgeMs);
      const bool core_valid = snapshot.valid &&
                              data_fresh &&
                              isfinite(snapshot.temperature_c) &&
                              isfinite(snapshot.humidity_pct) &&
                              isfinite(snapshot.pressure_hpa);

      const bool gas_valid = isfinite(snapshot.gas_resistance_kohm) && (snapshot.gas_resistance_kohm > 0.0f);

      result.data_ok = core_valid && gas_valid;

      if (result.data_ok)
      {
        break;
      }

      delay(80);
    }

    return result;
  }

  void printBootReadinessChecklist(const BootDiagStatus &boot_status, SensorManager &sensor_manager)
  {
    const SensorData snapshot = sensor_manager.getData();
    const uint32_t now_ms = millis();
    const bool data_fresh = (snapshot.last_update_ms != 0U) && ((now_ms - snapshot.last_update_ms) <= 5000U);
    const bool gas_valid = isfinite(snapshot.gas_resistance_kohm) && (snapshot.gas_resistance_kohm > 0.0f);

    const bool ready = boot_status.lcd_ok &&
                       boot_status.touch_ok &&
                       boot_status.sensor_ok &&
                       boot_status.data_ok &&
                       data_fresh &&
                       gas_valid;

    Serial.println("[BOOT] Production readiness checklist");
    Serial.printf("[BOOT]  LCD .......... %s\n", boot_status.lcd_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Touch ........ %s\n", boot_status.touch_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Sensor init .. %s\n", boot_status.sensor_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Gas Data ..... %s\n", (boot_status.data_ok && data_fresh) ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Gas data ..... %s\n", gas_valid ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Verdict ...... %s\n", ready ? "READY" : "DEGRADED");

    if (!boot_status.data_ok)
    {
      Serial.printf("[BOOT]  DEBUG: sensor valid=%d, temp=%f, hum=%f, pres=%f\n",
                    snapshot.valid,
                    snapshot.temperature_c,
                    snapshot.humidity_pct,
                    snapshot.pressure_hpa);
      Serial.printf("[BOOT]  DEBUG: gas_valid=%d, gas_res=%f, last_update=%lu\n",
                    gas_valid,
                    snapshot.gas_resistance_kohm,
                    static_cast<unsigned long>(snapshot.last_update_ms));
    }
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


  TaskHandle_t sensor_task_handle = nullptr;
  xTaskCreatePinnedToCore(SensorManager::taskEntry, "sensorTask", 8192, &sensor_manager, 2, &sensor_task_handle, 0);


  if (boot_status.sensor_ok)
  {
    const BootDataCheckResult check = verifySensorData(sensor_manager);
    boot_status.data_done = true;
    boot_status.data_ok = check.data_ok;
  }
  else
  {
    boot_status.data_done = true;
    boot_status.data_ok = false;
  }


  printBootReadinessChecklist(boot_status, sensor_manager);

  ui.bootDiagFinish(3000);

  ui.buildPages();

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