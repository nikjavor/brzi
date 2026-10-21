#include "ble.h"
#include "ble_protocol.h"

#include "esp_log.h"
#include "nvs_flash.h"
#include "host/ble_att.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/nimble_npl.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "BLE";
static const char DEVICE_NAME[] = "BRZI";

// f023e430-7ac9-4f6d-a40a-5a43b9181a01; NimBLE uses least-significant byte first.
static const ble_uuid128_t s_service_uuid = BLE_UUID128_INIT(
    0x01, 0x1a, 0x18, 0xb9, 0x43, 0x5a, 0x0a, 0xa4,
    0x6d, 0x4f, 0xc9, 0x7a, 0x30, 0xe4, 0x23, 0xf0);
// f023e430-7ac9-4f6d-a40a-5a43b9181a02
static const ble_uuid128_t s_telemetry_uuid = BLE_UUID128_INIT(
    0x02, 0x1a, 0x18, 0xb9, 0x43, 0x5a, 0x0a, 0xa4,
    0x6d, 0x4f, 0xc9, 0x7a, 0x30, 0xe4, 0x23, 0xf0);

// After startup, all state below is owned by the NimBLE host event queue.
static bool s_started;
static bool s_subscribed;
static uint8_t s_address_type;
static uint16_t s_connection = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_telemetry_handle;
static uint16_t s_sequence;
static struct ble_npl_callout s_publish_timer;
static uint8_t s_body[BLE_PROTOCOL_BODY_SIZE];

static int gap_event(struct ble_gap_event *event, void *arg);

static int telemetry_access(uint16_t connection, uint16_t handle,
                            struct ble_gatt_access_ctxt *context, void *arg)
{
    (void)connection;
    (void)handle;
    (void)context;
    (void)arg;
    return BLE_ATT_ERR_READ_NOT_PERMITTED;
}

static const struct ble_gatt_svc_def s_services[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_service_uuid.u,
        .characteristics = (const struct ble_gatt_chr_def[]) {
            {
                .uuid = &s_telemetry_uuid.u,
                .access_cb = telemetry_access,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &s_telemetry_handle,
            },
            {0},
        },
    },
    {0},
};

static void advertise(void)
{
    if (s_connection != BLE_HS_CONN_HANDLE_NONE || ble_gap_adv_active())
        return;

    // Flags + 128-bit service UUID + name occupy 27 of the available 31 bytes.
    const struct ble_hs_adv_fields fields = {
        .flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP,
        .name = (const uint8_t *)DEVICE_NAME,
        .name_len = sizeof(DEVICE_NAME) - 1,
        .name_is_complete = 1,
        .uuids128 = &s_service_uuid,
        .num_uuids128 = 1,
        .uuids128_is_complete = 1,
    };
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc == 0) {
        const struct ble_gap_adv_params params = {
            .conn_mode = BLE_GAP_CONN_MODE_UND,
            .disc_mode = BLE_GAP_DISC_MODE_GEN,
        };
        rc = ble_gap_adv_start(s_address_type, NULL, BLE_HS_FOREVER,
                               &params, gap_event, NULL);
    }
    if (rc != 0)
        ESP_LOGE(TAG, "Advertising failed: %d", rc);
}

static void stop_stream(void)
{
    s_subscribed = false;
    ble_npl_callout_stop(&s_publish_timer);
}

static void publish(struct ble_npl_event *event)
{
    (void)event;
    if (!s_subscribed || s_connection == BLE_HS_CONN_HANDLE_NONE)
        return;

    car_value_t values[CAR_DATA_COUNT];
    uint32_t captured_ms;
    if (car_data_get_values_snapshot(values, &captured_ms)) {
        ble_protocol_encode(values, captured_ms, s_body);
        uint16_t sequence = s_sequence++;
        uint16_t mtu = ble_att_mtu(s_connection);
        uint8_t packet[BLE_PROTOCOL_MAX_PACKET_SIZE];

        for (uint8_t index = 0; ; index++) {
            size_t length = ble_protocol_fragment(s_body, sequence, mtu, index, packet);
            if (length == 0)
                break;

            struct os_mbuf *buffer = ble_hs_mbuf_from_flat(packet, (uint16_t)length);
            if (buffer == NULL)
                break;
            // NimBLE consumes the buffer on success AND failure.
            if (ble_gatts_notify_custom(s_connection, s_telemetry_handle, buffer) != 0)
                break;
        }
    }

    // Congestion drops this snapshot. No CAN work or retry queue lives here.
    if (ble_npl_callout_reset(&s_publish_timer, ble_npl_time_ms_to_ticks32(100)) != BLE_NPL_OK)
        ESP_LOGE(TAG, "Could not schedule telemetry");
}

static int gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_connection = event->connect.conn_handle;
            ESP_LOGI(TAG, "Phone connected");
        } else {
            advertise();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        stop_stream();
        s_connection = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "Phone disconnected");
        advertise();
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.conn_handle == s_connection &&
            event->subscribe.attr_handle == s_telemetry_handle) {
            stop_stream();
            s_subscribed = event->subscribe.cur_notify;
            if (s_subscribed)
                publish(NULL);
        }
        break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        advertise();
        break;

    default:
        break;
    }
    return 0;
}

static void on_reset(int reason)
{
    stop_stream();
    s_connection = BLE_HS_CONN_HANDLE_NONE;
    ESP_LOGW(TAG, "Host reset: %d", reason);
}

static void on_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0)
        rc = ble_hs_id_infer_auto(0, &s_address_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Could not determine BLE address: %d", rc);
        return;
    }
    advertise();
}

static void host_task(void *arg)
{
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t ble_start(void)
{
    if (s_started)
        return ESP_ERR_INVALID_STATE;

    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        result = nvs_flash_erase();
        if (result == ESP_OK)
            result = nvs_flash_init();
    }
    if (result != ESP_OK)
        return result;

    result = nimble_port_init();
    if (result != ESP_OK)
        return result;

    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sync_cb = on_sync;
    ble_svc_gap_init();
    ble_svc_gatt_init();

    int rc = ble_svc_gap_device_name_set(DEVICE_NAME);
    if (rc == 0)
        rc = ble_gatts_count_cfg(s_services);
    if (rc == 0)
        rc = ble_gatts_add_svcs(s_services);
    if (rc == 0)
        rc = ble_npl_callout_init(&s_publish_timer, nimble_port_get_dflt_eventq(), publish, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Telemetry initialization failed: %d", rc);
        nimble_port_deinit();
        return ESP_FAIL;
    }

    s_started = true;
    nimble_port_freertos_init(host_task);
    return ESP_OK;
}
