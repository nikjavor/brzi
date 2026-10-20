#include <stdio.h>
#include "obd_twai.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"

static const char *TAG = "OBD_TWAI";

static bool twai_rx_cb(twai_node_handle_t, const twai_rx_done_event_data_t *, void *);
static esp_err_t detect_bitrate(void);
static void drain_rx_queue();
static bool send_pid_request(uint8_t, uint8_t);
static bool wait_response(uint8_t, uint8_t, uint8_t *, size_t *, TickType_t);
static bool decode_rpm(const uint8_t *, size_t, obd_decoded_value_t *);
static bool decode_speed(const uint8_t *, size_t, obd_decoded_value_t *);
static bool decode_throttle_position(const uint8_t *, size_t, obd_decoded_value_t *);

static QueueHandle_t s_data_queue = NULL;
static twai_node_handle_t s_node_hdl = NULL;

static obd_twai_config_t s_config;

static uint32_t s_detected_bitrate = 0;
static volatile uint32_t s_rx_count = 0;
static uint32_t s_rx_drop_count = 0;
static portMUX_TYPE s_stats_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct
{
    uint32_t id;
    bool ide;
    uint8_t len;
    uint8_t data[8];

} twai_rx_item_t;

static twai_event_callbacks_t s_twai_cbs = {
    .on_rx_done = twai_rx_cb,
};

static twai_onchip_node_config_t
make_node_config(uint32_t bitrate, bool listen_only)
{
    twai_onchip_node_config_t cfg = {
        .io_cfg.tx = s_config.tx_gpio,
        .io_cfg.rx = s_config.rx_gpio,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate = bitrate,
        .tx_queue_depth = 5,
        .flags.enable_listen_only = listen_only,
    };

    return cfg;
}

typedef bool (*obd_decoder_fn_t)(const uint8_t *data,
                                 size_t len,
                                 obd_decoded_value_t *out);

typedef struct
{
    obd_signal_t signal;
    uint8_t service;
    uint8_t pid;
    uint8_t min_response_len;
    obd_value_type_t value_type;
    obd_decoder_fn_t decoder;
} obd_signal_def_t;

static const obd_signal_def_t obd_signal_defs[OBD_SIGNAL_COUNT] = {
    [OBD_SIGNAL_RPM] = {
        .signal = OBD_SIGNAL_RPM,
        .service = 0x01,
        .pid = 0x0C,
        .min_response_len = 5,
        .value_type = OBD_VALUE_U16,
        .decoder = decode_rpm,
    },

    [OBD_SIGNAL_SPEED] = {
        .signal = OBD_SIGNAL_SPEED,
        .service = 0x01,
        .pid = 0x0D,
        .min_response_len = 4,
        .value_type = OBD_VALUE_U16,
        .decoder = decode_speed,
    },

    [OBD_SIGNAL_THROTTLE_POSITION] = {
        .signal = OBD_SIGNAL_THROTTLE_POSITION,
        .service = 0x01,
        .pid = 0x11,
        .min_response_len = 4,
        .value_type = OBD_VALUE_U16,
        .decoder = decode_throttle_position,
    },
};

esp_err_t obd_twai_start(const obd_twai_config_t *config)
{
    if (s_data_queue != NULL)
    {
        ESP_LOGI(TAG, "TWAI failed to start: Data queue should be NULL");
        return ESP_ERR_INVALID_STATE;
    }

    s_config = config == NULL ? OBD_TWAI_DEFAULT_CONFIG() : *config;

    esp_err_t err = detect_bitrate();
    if (err != ESP_OK)
    {
        ESP_LOGI(TAG, "TWAI failed to start: Bitrate detection failed");
        return err;
    }

    s_data_queue = xQueueCreate(s_config.queue_len, sizeof(twai_rx_item_t));
    if (s_data_queue == NULL)
    {
        ESP_LOGI(TAG, "TWAI failed to start: Failed to create data queue");
        return ESP_ERR_NO_MEM;
    }

    twai_onchip_node_config_t node_config = make_node_config(s_detected_bitrate, false);

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &s_node_hdl));
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node_hdl, &s_twai_cbs, NULL));
    ESP_ERROR_CHECK(twai_node_enable(s_node_hdl));

    ESP_LOGI(TAG, "TWAI started");
    return ESP_OK;
}

static bool twai_rx_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8];
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    if (ESP_OK != twai_node_receive_from_isr(handle, &rx_frame))
        return false;

    // receive ok, do something here
    s_rx_count++;

    if (s_data_queue == NULL)
        return false;

    twai_rx_item_t item = {
        .id = rx_frame.header.id,
        .ide = rx_frame.header.ide,
        .len = rx_frame.buffer_len <= 8 ? rx_frame.buffer_len : 8,
    };

    memcpy(&item.data, rx_frame.buffer, item.len);

    BaseType_t hp_task_woken = pdFALSE;

    if (!item.ide && item.id >= 0x7E8 && item.id <= 0x7EF)
        if (xQueueSendFromISR(s_data_queue, &item, &hp_task_woken) != pdTRUE)
        {
            portENTER_CRITICAL_ISR(&s_stats_lock);
            s_rx_drop_count++;
            portEXIT_CRITICAL_ISR(&s_stats_lock);
        }

    return hp_task_woken == pdTRUE;
}

static bool try_bitrate(uint32_t bitrate)
{
    twai_onchip_node_config_t node_config = make_node_config(bitrate, true);

    esp_err_t err = twai_new_node_onchip(&node_config, &s_node_hdl);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "try_bitrate %lu: create node failed: %s", bitrate, esp_err_to_name(err));
        s_node_hdl = NULL;
        return false;
    }

    err = twai_node_register_event_callbacks(s_node_hdl, &s_twai_cbs, NULL);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "try_bitrate %lu: register callbacks failed: %s", bitrate, esp_err_to_name(err));
        twai_node_delete(s_node_hdl);
        s_node_hdl = NULL;
        return false;
    }

    s_rx_count = 0;

    err = twai_node_enable(s_node_hdl);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "try_bitrate %lu: node enable failed: %s", bitrate, esp_err_to_name(err));
        twai_node_delete(s_node_hdl);
        s_node_hdl = NULL;
        return false;
    }

    ESP_LOGI(TAG, "Waiting for can events...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // wait for can events

    twai_node_disable(s_node_hdl);
    twai_node_delete(s_node_hdl);
    s_node_hdl = NULL;

    ESP_LOGI(TAG, "try_bitrate %lu: rx_count=%lu", (unsigned long)bitrate, (unsigned long)s_rx_count);

    return s_rx_count > 0;
}

static esp_err_t detect_bitrate(void)
{
    while (1)
    {
        ESP_LOGI(TAG, "Detecting bitrate...");

        if (try_bitrate(250000))
        {
            s_detected_bitrate = 250000;
            break;
        }
        else if (try_bitrate(500000))
        {
            s_detected_bitrate = 500000;
            break;
        }
    }

    ESP_LOGI(TAG, "Detected bitrate: %lu", s_detected_bitrate);
    return ESP_OK;
}

bool obd_twai_query_signal(obd_signal_t signal, obd_decoded_value_t *out, TickType_t timeout)
{
    if (out == NULL || signal >= OBD_SIGNAL_COUNT)
    {
        return false;
    }

    // LOOKUP SIGNAL DEFINITION
    const obd_signal_def_t *def = &obd_signal_defs[signal];

    if (def->decoder == NULL)
        return false;

    uint8_t raw[8];
    size_t raw_len = 0;

    // DRAIN RX FRAMES
    drain_rx_queue();

    // SEND OBD REQUEST
    if (!send_pid_request(def->service, def->pid))
    {
        return false;
    }

    // WAIT FOR CORRECT MESSAGE
    if (!wait_response(def->service, def->pid, raw, &raw_len, timeout))
    {
        return false;
    }

    if (raw_len < def->min_response_len)
    {
        return false;
    }

    // DECODE VALUE
    return def->decoder(raw, raw_len, out);
}

static void drain_rx_queue()
{
    twai_rx_item_t dummy;
    while (xQueueReceive(s_data_queue, &dummy, 0) == pdTRUE)
    {
    }
}

static bool send_pid_request(uint8_t service, uint8_t pid)
{
    uint8_t send_buff[8] = {0x02, service, pid}; // number of following bytes, service, pid
    twai_frame_t tx_msg = {
        .header.id = 0x7df,
        .header.ide = false,
        .buffer = send_buff,
        .buffer_len = sizeof(send_buff),
    };

    if (twai_node_transmit_wait_all_done(s_node_hdl, 0) != ESP_OK)
        return false;

    if (twai_node_transmit(s_node_hdl, &tx_msg, 10) != ESP_OK)
        return false;

    return twai_node_transmit_wait_all_done(s_node_hdl, 10) == ESP_OK;
}

static bool wait_response(uint8_t service, uint8_t pid, uint8_t *out, size_t *out_len, TickType_t timeout)
{
    twai_rx_item_t recv_item;
    TickType_t start = xTaskGetTickCount();

    while (1)
    {
        TickType_t elapsed = xTaskGetTickCount() - start;
        if (elapsed >= timeout)
            return false;

        TickType_t remaining = timeout - elapsed;

        if (xQueueReceive(s_data_queue, &recv_item, remaining) != pdTRUE)
            return false;

        if (recv_item.ide ||
            recv_item.id < 0x7E8 ||
            recv_item.id > 0x7EF ||
            recv_item.len < 3)
        {
            continue;
        }

        const uint8_t *recv_buff = recv_item.data;

        if (recv_buff[1] != (uint8_t)(service + 0x40) ||
            recv_buff[2] != pid)
        {
            continue;
        }

        memcpy(out, recv_buff, recv_item.len);
        *out_len = recv_item.len;

        ESP_LOG_BUFFER_HEX_LEVEL(TAG, recv_buff, recv_item.len, ESP_LOG_DEBUG);

        return true;
    }

    return false;
}

uint32_t obd_twai_get_rx_drop_count(void)
{
    portENTER_CRITICAL(&s_stats_lock);
    uint32_t count = s_rx_drop_count;
    portEXIT_CRITICAL(&s_stats_lock);

    return count;
}

static bool decode_rpm(const uint8_t *data,
                       size_t len,
                       obd_decoded_value_t *out)
{
    if (len < 5)
    {
        return false;
    }

    uint8_t A = data[3];
    uint8_t B = data[4];

    out->type = OBD_VALUE_U16;
    out->value.u16 = ((uint16_t)A * 256u + B) / 4u;

    return true;
}
static bool decode_speed(const uint8_t *data,
                         size_t len,
                         obd_decoded_value_t *out)
{
    if (len < 4)
    {
        return false;
    }

    uint8_t A = data[3];

    out->type = OBD_VALUE_U16;
    out->value.u16 = A;

    return true;
}
static bool decode_throttle_position(const uint8_t *data,
                                     size_t len,
                                     obd_decoded_value_t *out)
{
    if (len < 4)
    {
        return false;
    }

    uint8_t A = data[3];

    out->type = OBD_VALUE_U16;
    out->value.u16 = ((uint16_t)A * 100u) / 255u;

    return true;
}

