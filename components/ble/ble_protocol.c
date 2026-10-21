#include "ble_protocol.h"

#include <string.h>

_Static_assert((int)CAR_DATA_COUNT == (int)BLE_PROTOCOL_SIGNAL_COUNT,
               "Update the BLE protocol version and layout when adding readings");

static void write_u16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void write_u32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

void ble_protocol_encode(const car_value_t values[CAR_DATA_COUNT], uint32_t captured_ms,
                         uint8_t body[BLE_PROTOCOL_BODY_SIZE])
{
    memset(body, 0, BLE_PROTOCOL_BODY_SIZE);
    write_u32(body, captured_ms);

    for (unsigned i = 0; i < BLE_PROTOCOL_SIGNAL_COUNT; i++) {
        const car_value_t *value = &values[i];
        if (!value->valid)
            continue;

        body[4 + i / 8] |= (uint8_t)(1u << (i % 8));
        uint8_t *record = body + 4 + BLE_PROTOCOL_VALIDITY_SIZE + i * BLE_PROTOCOL_RECORD_SIZE;
        uint32_t age = captured_ms - value->timestamp_ms;
        write_u32(record, (uint32_t)value->value);
        write_u16(record + 4, age > UINT16_MAX ? UINT16_MAX : (uint16_t)age);
    }
}

size_t ble_protocol_fragment(const uint8_t body[BLE_PROTOCOL_BODY_SIZE], uint16_t sequence,
                             uint16_t mtu, uint8_t index,
                             uint8_t packet[BLE_PROTOCOL_MAX_PACKET_SIZE])
{
    if (mtu < 23)
        return 0;
    if (mtu > BLE_PROTOCOL_MAX_MTU)
        mtu = BLE_PROTOCOL_MAX_MTU;

    size_t capacity = mtu - 3 - BLE_PROTOCOL_HEADER_SIZE;
    size_t count = (BLE_PROTOCOL_BODY_SIZE + capacity - 1) / capacity;
    if (index >= count)
        return 0;

    size_t offset = index * capacity;
    size_t length = BLE_PROTOCOL_BODY_SIZE - offset;
    if (length > capacity)
        length = capacity;

    packet[0] = BLE_PROTOCOL_VERSION;
    write_u16(packet + 1, sequence);
    packet[3] = index;
    packet[4] = (uint8_t)count;
    memcpy(packet + BLE_PROTOCOL_HEADER_SIZE, body + offset, length);
    return BLE_PROTOCOL_HEADER_SIZE + length;
}
