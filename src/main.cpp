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
    bool iaq_ok = false;
  };

  BootDataCheckResult verifyDataAndIaq(SensorManager &sensor_manager)
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

      delay(80);
    }

    return result;
  }

  void printBootReadinessChecklist(const BootDiagStatus &boot_status, SensorManager &sensor_manager)
  {
    const SensorData snapshot = sensor_manager.getData();
    const uint32_t now_ms = millis();
    const bool data_fresh = (snapshot.last_update_ms != 0U) && ((now_ms - snapshot.last_update_ms) <= 5000U);

    const bool iaq_available = isfinite(snapshot.iaq) || isfinite(snapshot.iaq_static);
    const bool iaq_effective_ok = isfinite(snapshot.iaq_effective) && (snapshot.iaq_effective >= 0.0f) && (snapshot.iaq_effective <= 500.0f);
    const bool model_metrics_ok = (snapshot.iaq_model_confidence <= 100U) && (snapshot.iaq_model_state <= 3U);

    const bool ready = boot_status.lcd_ok &&
                       boot_status.touch_ok &&
                       boot_status.sensor_ok &&
                       boot_status.data_ok &&
                       boot_status.iaq_ok &&
                       data_fresh &&
                       iaq_available &&
                       iaq_effective_ok &&
                       model_metrics_ok;

    Serial.println("[BOOT] Production readiness checklist");
    Serial.printf("[BOOT]  LCD .......... %s\n", boot_status.lcd_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Touch ........ %s\n", boot_status.touch_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Sensor init .. %s\n", boot_status.sensor_ok ? "OK" : "FAIL");
    Serial.printf("[BOOT]  Data fresh ... %s\n", (boot_status.data_ok && data_fresh) ? "OK" : "FAIL");
    Serial.printf("[BOOT]  IAQ core ..... %s\n", (boot_status.iaq_ok && iaq_available) ? "OK" : "FAIL");
    Serial.printf("[BOOT]  IAQ model .... %s (state=%u conf=%u%%)\n",
                  (iaq_effective_ok && model_metrics_ok) ? "OK" : "FAIL",
                  snapshot.iaq_model_state,
                  snapshot.iaq_model_confidence);
    Serial.printf("[BOOT]  Verdict ...... %s\n", ready ? "READY" : "DEGRADED");
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
    const BootDataCheckResult check = verifyDataAndIaq(sensor_manager);
    boot_status.data_done = true;
    boot_status.data_ok = check.data_ok;
    boot_status.iaq_done = true;
    boot_status.iaq_ok = check.iaq_ok;

    const SensorData snapshot = sensor_manager.getData();
    const bool model_metrics_ok = isfinite(snapshot.iaq_effective) &&
                                  (snapshot.iaq_effective >= 0.0f) &&
                                  (snapshot.iaq_effective <= 500.0f) &&
                                  (snapshot.iaq_model_confidence <= 100U) &&
                                  (snapshot.iaq_model_state <= 3U);
    boot_status.model_done = true;
    boot_status.model_ok = model_metrics_ok;

    ui.bootDiagUpdate(boot_status);
  }
  else
  {
    boot_status.data_done = true;
    boot_status.data_ok = false;
    boot_status.iaq_done = true;
    boot_status.iaq_ok = false;
    boot_status.model_done = true;
    boot_status.model_ok = false;
    ui.bootDiagUpdate(boot_status);
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