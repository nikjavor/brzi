#include <stdio.h>
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

static obd_pid_job_t pid_jobs[] = {
    // 10 Hz
    {OBD_SIGNAL_RPM, 100, 50, PID_PRIO_FAST, 0, 0, 0, true},

    // 5 Hz
    {OBD_SIGNAL_SPEED, 200, 50, PID_PRIO_FAST, 0, 0, 0, true},
    {OBD_SIGNAL_THROTTLE_POSITION, 200, 50, PID_PRIO_MEDIUM, 0, 0, 0, true},
};

static const char *signal_name(obd_signal_t signal)
{
  switch (signal)
  {
  case OBD_SIGNAL_RPM:
    return "RPM";
  case OBD_SIGNAL_SPEED:
    return "SPEED";
  case OBD_SIGNAL_THROTTLE_POSITION:
    return "THROTTLE";
  default:
    return "UNKNOWN";
  }
}

static obd_pid_job_t *pick_next_job(uint32_t now_ms)
{
  obd_pid_job_t *best = NULL;

  for (size_t i = 0; i < (sizeof(pid_jobs) / sizeof(pid_jobs[0])); i++)
  {
    obd_pid_job_t *job = &pid_jobs[i];
    if (!job->enabled)
    {
      continue;
    }

    if ((int32_t)(now_ms - job->next_due_ms) < 0)
    {
      continue;
    }

    if (best == NULL ||
        job->priority < best->priority ||
        (job->priority == best->priority &&
         (int32_t)(job->next_due_ms - best->next_due_ms) < 0))
    {
      best = job;
    }
  }

  return best;
}

static void scheduler_task(void *arg)
{
  uint32_t now = 0;
  uint32_t last_summary_ms = 0;
  obd_pid_job_t *job = NULL;

  while (1)
  {
    now = pdTICKS_TO_MS(xTaskGetTickCount());

    if ((now - last_summary_ms) >= 100) // Summary logging interval
    {
      last_summary_ms = now;

      car_data_t car;
      if (car_data_get_snapshot(&car))
      {
        ESP_LOGI(TAG, "RPM=%d(v%d) SPEED=%d(v%d) THR=%d(v%d)",
                 (int)car.engine.rpm.value, car.engine.rpm.valid,
                 (int)car.engine.speed_kph.value, car.engine.speed_kph.valid,
                 (int)car.engine.throttle_pct.value, car.engine.throttle_pct.valid);

        ESP_LOGI(TAG, "RX queue drops=%lu",
                 (unsigned long)obd_twai_get_rx_drop_count());
      }
    }

    job = pick_next_job(now);

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
      job->last_success_ms = now;
      job->next_due_ms = now + job->period_ms;
      job->fail_count = 0;

      ESP_LOGD(TAG, "%s = %d", signal_name(job->signal), (int)value.value.u16);

      switch (job->signal)
      {
      case OBD_SIGNAL_RPM:
        car_data_update(CAR_DATA_RPM, (int32_t)(value.value.u16), now);
        break;

      case OBD_SIGNAL_SPEED:
        car_data_update(CAR_DATA_SPEED_KPH, (int32_t)(value.value.u16), now);
        break;

      case OBD_SIGNAL_THROTTLE_POSITION:
        car_data_update(CAR_DATA_THROTTLE_PCT, (int32_t)(value.value.u16), now);
        break;

      default:
        break;
      }
    }
    else
    {
      if (job->fail_count < UINT8_MAX)
      {
        job->fail_count++;
      }

      ESP_LOGW(TAG, "%s query failed (fail_count=%u)", signal_name(job->signal), job->fail_count);

      job->next_due_ms = now + job->period_ms + (job->fail_count * 50);
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // TODO - change to wait untill next due pid
  }
}

esp_err_t obd_scheduler_start(void)
{
  BaseType_t res = xTaskCreate(
      scheduler_task,
      "scheduler_task",
      2048,
      NULL,
      5,
      NULL);

  if (res != pdPASS)
  {
    ESP_LOGW(TAG, "OBD scheduler failed to start: Failed to create scheduler task");
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "OBD scheduler started");
  return ESP_OK;
}