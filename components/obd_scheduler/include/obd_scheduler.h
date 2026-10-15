#pragma once

#include "obd_twai.h"

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PID_PRIO_FAST,
    PID_PRIO_MEDIUM,
    PID_PRIO_SLOW,
    PID_PRIO_ON_DEMAND
} obd_pid_priority_t;

typedef struct {
    obd_signal_t signal;

    uint16_t period_ms;
    uint16_t timeout_ms;
    obd_pid_priority_t priority;

    uint32_t next_due_ms;
    uint32_t last_success_ms;
    uint8_t fail_count;
    bool enabled;
} obd_pid_job_t;

esp_err_t obd_scheduler_start(void);
