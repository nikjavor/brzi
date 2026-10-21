#pragma once

#include <stddef.h>
#include <stdint.h>

#include "car_data.h"

enum {
    BLE_PROTOCOL_VERSION = 1,
    BLE_PROTOCOL_SIGNAL_COUNT = 35,
    BLE_PROTOCOL_HEADER_SIZE = 5,
    BLE_PROTOCOL_VALIDITY_SIZE = 5,
    BLE_PROTOCOL_RECORD_SIZE = 6,
    BLE_PROTOCOL_BODY_SIZE = 4 + BLE_PROTOCOL_VALIDITY_SIZE +
                             BLE_PROTOCOL_SIGNAL_COUNT * BLE_PROTOCOL_RECORD_SIZE,
    BLE_PROTOCOL_MAX_MTU = 247,
    BLE_PROTOCOL_MAX_PACKET_SIZE = BLE_PROTOCOL_MAX_MTU - 3,
};

// Internal, SDK-independent wire encoder. Output buffers have the sizes above.
void ble_protocol_encode(const car_value_t values[CAR_DATA_COUNT], uint32_t captured_ms,
                         uint8_t body[BLE_PROTOCOL_BODY_SIZE]);

// Returns packet length, or zero for an invalid MTU/index. MTUs above 247 are capped.
size_t ble_protocol_fragment(const uint8_t body[BLE_PROTOCOL_BODY_SIZE], uint16_t sequence,
                             uint16_t mtu, uint8_t index,
                             uint8_t packet[BLE_PROTOCOL_MAX_PACKET_SIZE]);
