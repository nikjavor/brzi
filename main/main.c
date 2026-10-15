#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "obd_twai.h"
#include "car_data.h"
#include "obd_scheduler.h"

static const char *TAG = "MAIN";


void app_main(void)
{
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(TAG, "Start initialization");

  car_data_init();

  obd_twai_config_t obd_config = OBD_TWAI_DEFAULT_CONFIG();
  ESP_ERROR_CHECK(obd_twai_start(&obd_config));

  ESP_ERROR_CHECK(obd_scheduler_start());

  ESP_LOGI(TAG, "Brzee initialized");
}
