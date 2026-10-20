#include "obd_scheduler.h"
#include "obd_twai.h"
#include "car_data.h"

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "OBD_SCHED";

#define JOB(sig, period, prio) \
  {.signal = sig, .period_ms = period, .timeout_ms = 50, .priority = prio, .enabled = true}

// Per-signal minimum intervals; queries share one serial CAN requester.
static obd_pid_job_t pid_jobs[] = {
    JOB(OBD_SIGNAL_RPM, 100, PID_PRIO_FAST),
    JOB(OBD_SIGNAL_SPEED, 200, PID_PRIO_FAST),
    JOB(OBD_SIGNAL_PEDAL_POSITION_D, 250, PID_PRIO_FAST),
    JOB(OBD_SIGNAL_RELATIVE_PEDAL_POSITION, 250, PID_PRIO_FAST),
    JOB(OBD_SIGNAL_THROTTLE_POSITION, 500, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_MANIFOLD_PRESSURE, 500, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_ENGINE_LOAD, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_MASS_AIR_FLOW, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_FUEL_RATE, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_COOLANT_TEMP, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_OIL_TEMP, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_INTAKE_AIR_TEMP, 2000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_BATTERY_VOLTAGE, 2000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_BAROMETRIC_PRESSURE, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_FUEL_LEVEL, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_AMBIENT_TEMPERATURE, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_RELATIVE_THROTTLE_POSITION, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_THROTTLE_ACTUATOR, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_PEDAL_POSITION_E, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_PEDAL_POSITION_F, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_THROTTLE_POSITION_B, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_THROTTLE_POSITION_C, 1000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_1, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_2, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_LONG_FUEL_TRIM_BANK_1, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_LONG_FUEL_TRIM_BANK_2, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_ENGINE_RUN_TIME, 5000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_TIMING_ADVANCE, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_DEMANDED_TORQUE, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_ACTUAL_TORQUE, 1000, PID_PRIO_MEDIUM),
    JOB(OBD_SIGNAL_REFERENCE_TORQUE, 30000, PID_PRIO_SLOW),
    JOB(OBD_SIGNAL_FUEL_RAIL_PRESSURE, 1000, PID_PRIO_MEDIUM),
};

#undef JOB

static const car_data_id_t signal_data_ids[OBD_SIGNAL_COUNT] = {
    [OBD_SIGNAL_ENGINE_LOAD] = CAR_DATA_ENGINE_LOAD_PCT,
    [OBD_SIGNAL_COOLANT_TEMP] = CAR_DATA_COOLANT_TEMP_C,
    [OBD_SIGNAL_RPM] = CAR_DATA_RPM,
    [OBD_SIGNAL_SPEED] = CAR_DATA_SPEED_KPH,
    [OBD_SIGNAL_THROTTLE_POSITION] = CAR_DATA_THROTTLE_PCT,
    [OBD_SIGNAL_FUEL_LEVEL] = CAR_DATA_FUEL_LEVEL_PCT,
    [OBD_SIGNAL_BAROMETRIC_PRESSURE] = CAR_DATA_BAROMETRIC_PRESSURE_KPA,
    [OBD_SIGNAL_BATTERY_VOLTAGE] = CAR_DATA_MODULE_VOLTAGE_MV,
    [OBD_SIGNAL_RELATIVE_THROTTLE_POSITION] = CAR_DATA_RELATIVE_THROTTLE_PCT,
    [OBD_SIGNAL_AMBIENT_TEMPERATURE] = CAR_DATA_AMBIENT_TEMP_C,
    [OBD_SIGNAL_THROTTLE_ACTUATOR] = CAR_DATA_THROTTLE_ACTUATOR_PCT,
    [OBD_SIGNAL_OIL_TEMP] = CAR_DATA_OIL_TEMP_C,
    [OBD_SIGNAL_MANIFOLD_PRESSURE] = CAR_DATA_MANIFOLD_PRESSURE_KPA,
    [OBD_SIGNAL_INTAKE_AIR_TEMP] = CAR_DATA_INTAKE_AIR_TEMP_C,
    [OBD_SIGNAL_MASS_AIR_FLOW] = CAR_DATA_MASS_AIR_FLOW_MG_S,
    [OBD_SIGNAL_FUEL_RATE] = CAR_DATA_FUEL_RATE_ML_H,
    [OBD_SIGNAL_PEDAL_POSITION_D] = CAR_DATA_PEDAL_POSITION_D_PCT,
    [OBD_SIGNAL_PEDAL_POSITION_E] = CAR_DATA_PEDAL_POSITION_E_PCT,
    [OBD_SIGNAL_PEDAL_POSITION_F] = CAR_DATA_PEDAL_POSITION_F_PCT,
    [OBD_SIGNAL_RELATIVE_PEDAL_POSITION] = CAR_DATA_RELATIVE_PEDAL_POSITION_PCT,
    [OBD_SIGNAL_THROTTLE_POSITION_B] = CAR_DATA_THROTTLE_POSITION_B_PCT,
    [OBD_SIGNAL_THROTTLE_POSITION_C] = CAR_DATA_THROTTLE_POSITION_C_PCT,
    [OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_1] = CAR_DATA_SHORT_FUEL_TRIM_BANK_1_PCT_X100,
    [OBD_SIGNAL_LONG_FUEL_TRIM_BANK_1] = CAR_DATA_LONG_FUEL_TRIM_BANK_1_PCT_X100,
    [OBD_SIGNAL_SHORT_FUEL_TRIM_BANK_2] = CAR_DATA_SHORT_FUEL_TRIM_BANK_2_PCT_X100,
    [OBD_SIGNAL_LONG_FUEL_TRIM_BANK_2] = CAR_DATA_LONG_FUEL_TRIM_BANK_2_PCT_X100,
    [OBD_SIGNAL_ENGINE_RUN_TIME] = CAR_DATA_ENGINE_RUN_TIME_S,
    [OBD_SIGNAL_TIMING_ADVANCE] = CAR_DATA_TIMING_ADVANCE_DEG_X10,
    [OBD_SIGNAL_DEMANDED_TORQUE] = CAR_DATA_DEMANDED_TORQUE_PCT,
    [OBD_SIGNAL_ACTUAL_TORQUE] = CAR_DATA_ACTUAL_TORQUE_PCT,
    [OBD_SIGNAL_REFERENCE_TORQUE] = CAR_DATA_REFERENCE_TORQUE_NM,
    [OBD_SIGNAL_FUEL_RAIL_PRESSURE] = CAR_DATA_FUEL_RAIL_PRESSURE_KPA,
};

static void apply_signal_support(void)
{
  for (size_t i = 0; i < sizeof(pid_jobs) / sizeof(pid_jobs[0]); i++)
  {
    obd_pid_job_t *job = &pid_jobs[i];
    if (obd_twai_signal_support(job->signal) == OBD_SUPPORT_UNSUPPORTED)
    {
      if (job->enabled)
        ESP_LOGI(TAG, "%s unsupported; skipping", obd_signal_get_info(job->signal)->name);
      job->enabled = false;
      car_data_invalidate(signal_data_ids[job->signal]);
    }
  }
}

static obd_pid_job_t *pick_next_job(uint32_t now_ms)
{
  obd_pid_job_t *best = NULL;
  for (size_t i = 0; i < sizeof(pid_jobs) / sizeof(pid_jobs[0]); i++)
  {
    obd_pid_job_t *job = &pid_jobs[i];
    if (!job->enabled || (int32_t)(now_ms - job->next_due_ms) < 0)
      continue;

    // Service the oldest deadline so slow signals get a turn, too.
    if (best == NULL || (int32_t)(job->next_due_ms - best->next_due_ms) < 0 ||
        (job->next_due_ms == best->next_due_ms && job->priority < best->priority))
      best = job;
  }
  return best;
}

static void log_summary(uint32_t now)
{
  ESP_LOGI(TAG, "Live data (integer units), RX drops=%lu",
           (unsigned long)obd_twai_get_rx_drop_count());
  for (size_t i = 0; i < sizeof(pid_jobs) / sizeof(pid_jobs[0]); i++)
  {
    const obd_pid_job_t *job = &pid_jobs[i];
    if (!job->enabled)
      continue;
    const obd_signal_info_t *info = obd_signal_get_info(job->signal);
    car_value_t value;
    if (car_data_get_full(signal_data_ids[job->signal], &value))
      ESP_LOGI(TAG, "%s=%ld %s (age=%lu ms)", info->name, (long)value.value,
               info->unit, (unsigned long)(now - value.timestamp_ms));
    else
      ESP_LOGI(TAG, "%s=N/A", info->name);
  }

  const car_data_id_t derived_ids[] = {
      CAR_DATA_BOOST_PRESSURE_KPA, CAR_DATA_FUEL_CONSUMPTION_ML_100KM, CAR_DATA_ACTUAL_TORQUE_NM};
  const char *names[] = {"BOOST kPa", "CONSUMPTION mL/100km", "TORQUE Nm"};
  for (size_t i = 0; i < sizeof(derived_ids) / sizeof(derived_ids[0]); i++)
  {
    car_value_t value;
    if (car_data_get_full(derived_ids[i], &value))
      ESP_LOGI(TAG, "%s=%ld", names[i], (long)value.value);
    else
      ESP_LOGI(TAG, "%s=N/A", names[i]);
  }
}

static void scheduler_task(void *arg)
{
  bool support_complete = obd_twai_discover_supported_signals(pdMS_TO_TICKS(100));
  apply_signal_support();
  ESP_LOGI(TAG, "PID support scan %s", support_complete ? "complete" : "partial; will retry");
  uint32_t now = pdTICKS_TO_MS(xTaskGetTickCount());
  uint32_t last_summary_ms = now;
  uint32_t last_discovery_ms = now;
  for (size_t i = 0; i < sizeof(pid_jobs) / sizeof(pid_jobs[0]); i++)
    pid_jobs[i].next_due_ms = now + (uint32_t)i * 10u;

  while (1)
  {
    now = pdTICKS_TO_MS(xTaskGetTickCount());
    if (!support_complete && now - last_discovery_ms >= 10000u)
    {
      support_complete = obd_twai_discover_supported_signals(pdMS_TO_TICKS(100));
      apply_signal_support();
      now = pdTICKS_TO_MS(xTaskGetTickCount());
      last_discovery_ms = now;
    }

    if (now - last_summary_ms >= 1000u)
    {
      last_summary_ms = now;
      log_summary(now);
      now = pdTICKS_TO_MS(xTaskGetTickCount());
    }

    obd_pid_job_t *job = pick_next_job(now);
    if (job == NULL)
    {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    obd_decoded_value_t value;
    bool res = obd_twai_query_signal(job->signal, &value, pdMS_TO_TICKS(job->timeout_ms));
    now = pdTICKS_TO_MS(xTaskGetTickCount());
    if (res)
    {
      int32_t stored;
      switch (value.type)
      {
      case OBD_VALUE_U16:
        stored = value.value.u16;
        break;
      case OBD_VALUE_I16:
        stored = value.value.i16;
        break;
      case OBD_VALUE_U32:
        stored = (int32_t)value.value.u32;
        break;
      default:
        res = false;
        stored = 0;
        break;
      }
      if (res)
      {
        car_data_update(signal_data_ids[job->signal], stored, now);
        job->last_success_ms = now;
        job->next_due_ms = now + job->period_ms;
        job->fail_count = 0;
      }
    }
    if (!res)
    {
      car_data_invalidate(signal_data_ids[job->signal]);
      if (job->fail_count < UINT8_MAX)
        job->fail_count++;
      ESP_LOGW(TAG, "%s query failed (fail_count=%u)",
               obd_signal_get_info(job->signal)->name, job->fail_count);
      job->next_due_ms = now + job->period_ms + (job->fail_count * 50u);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

esp_err_t obd_scheduler_start(void)
{
  BaseType_t res = xTaskCreate(scheduler_task, "scheduler_task", 4096, NULL, 5, NULL);
  if (res != pdPASS)
  {
    ESP_LOGW(TAG, "OBD scheduler failed to start: Failed to create scheduler task");
    return ESP_FAIL;
  }
  ESP_LOGI(TAG, "OBD scheduler started");
  return ESP_OK;
}
