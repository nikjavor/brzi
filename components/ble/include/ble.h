#pragma once

#include "esp_err.h"

// Starts the BRZI telemetry peripheral. Call once after car_data_init().
esp_err_t ble_start(void);
