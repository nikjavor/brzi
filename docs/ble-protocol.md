# BRZI BLE telemetry, version 1

BRZI publishes a complete snapshot of 35 readings immediately after subscription,
then every 100 ms. Each reading is the latest available OBD or derived value;
BLE does not change the OBD polling intervals. A slow ECU can update less often
than the stream. Snapshot timestamps describe controller uptime, not UTC.

## Connection

| Item | Value |
| --- | --- |
| Advertised name | `BRZI` |
| Primary service UUID | `f023e430-7ac9-4f6d-a40a-5a43b9181a01` |
| Telemetry characteristic UUID | `f023e430-7ac9-4f6d-a40a-5a43b9181a02` |
| Characteristic properties | Notify |
| Subscription | Standard Client Characteristic Configuration Descriptor, UUID `2902` |
| Concurrent phones | 1 |
| Pairing / encryption / bonding | None |
| Preferred ATT MTU | 247 |

Filter scans by service UUID, connect, discover the characteristic, and subscribe.
Enable notifications through Core Bluetooth on iOS. On Android, enable local
notifications and write `01 00` to the CCCD; request an MTU of at least 247 before
subscribing when possible. Use the negotiated MTU, since the requested value is
not guaranteed. Both platforms must also accept fragmented snapshots.

Streaming stops on unsubscribe, disconnect, or host reset. Advertising restarts
after disconnect or host synchronization. The next subscription sends a fresh
snapshot. The telemetry characteristic exposes its standard subscription
descriptor for client writes; there are no vehicle-control commands.

## Notification framing

All multibyte numbers are little-endian, without padding. Every notification
starts with this five-byte header, even when the snapshot needs one fragment:

| Notification offset | Size | Field |
| --- | --- | --- |
| 0 | 1 | Protocol version, always `1` |
| 1 | 2 | Unsigned snapshot sequence number |
| 3 | 1 | Zero-based fragment index |
| 4 | 1 | Total fragment count |
| 5 | Remaining | Consecutive bytes of the snapshot body |

The sequence increments for every attempted snapshot, including attempts dropped
because BLE buffers are exhausted. It wraps from 65535 to 0 and can restart after
a controller reboot. All fragments of one snapshot share a sequence and count.

The controller uses `min(negotiated_mtu, 247) - 3 - 5` body bytes per fragment.
Every fragment except the last fills that capacity. Body records may cross
fragment boundaries. Concatenate fragment payloads in index order before decoding.

| ATT MTU | Body bytes per fragment | Fragments | Total notification-value bytes per snapshot |
| --- | --- | --- | --- |
| 23 | 15 | 15 | 294 |
| 185 | 177 | 2 | 229 |
| 247 or higher | 239 | 1 | 224 |

At MTU 247, the stream carries 2,240 bytes per second before Bluetooth overhead.
Under congestion, firmware abandons the rest of a failed snapshot and tries a
fresh one next tick. It does not retry old snapshots or maintain a transmit queue.
Small MTUs require more notifications, and sustained throughput depends on the
phone's connection parameters. Sequence gaps are expected and need no requests
for retransmission.

## Snapshot body

The assembled version 1 body is exactly **219 bytes**:

| Body offset | Size | Field |
| --- | --- | --- |
| 0 | 4 | Unsigned controller capture time in milliseconds |
| 4 | 5 | Validity bitmap, IDs 0 through 34 |
| `9 + 6 * id` | 4 | Signed, two's-complement 32-bit integer value |
| `13 + 6 * id` | 2 | Unsigned sample age in milliseconds |

Bit `(id % 8)` in bitmap byte `(id / 8)` is the validity of that reading. Bits
35–39 are zero. A cleared bit means unavailable/invalid, not a numeric zero;
its value and age bytes are also zero. Version 1 does not distinguish unsupported,
never received, failed-query, and invalid-derived readings.

For a valid reading, age is `min(uint32(captured_ms - timestamp_ms), 65535)`.
65535 means **at least 65.535 seconds old**. Timestamp subtraction handles the
32-bit millisecond counter wrapping; capture time itself wraps after roughly
49.7 days. Refresh-derived-value rules and scheduler invalidation remain the
controller's responsibility. BLE preserves validity rather than applying another
staleness policy to individual signals.

The record order is fixed by the following IDs, independent of the layout of
C structs. Adding, removing, or changing records or units requires a new protocol
version. A phone must reject unknown versions before interpreting the body.

| ID | Reading | Raw unit | Example display conversion |
| --- | --- | --- | --- |
| 0 | Engine RPM | rpm | unchanged |
| 1 | Vehicle speed | km/h | unchanged |
| 2 | Coolant temperature | °C | unchanged |
| 3 | Throttle position | % | unchanged |
| 4 | Engine load | % | unchanged |
| 5 | Fuel level | % | unchanged |
| 6 | Barometric pressure | kPa | unchanged |
| 7 | Control module supply voltage | mV | / 1000 → V |
| 8 | Relative throttle position | % | unchanged |
| 9 | Ambient temperature | °C | unchanged |
| 10 | Commanded throttle actuator | % | unchanged |
| 11 | Oil temperature | °C | unchanged |
| 12 | Intake manifold absolute pressure | kPa | unchanged |
| 13 | Intake air temperature | °C | unchanged |
| 14 | Mass airflow | mg/s | / 1000 → g/s |
| 15 | Fuel rate | mL/h | / 1000 → L/h |
| 16 | Accelerator pedal D | % | unchanged |
| 17 | Accelerator pedal E | % | unchanged |
| 18 | Accelerator pedal F | % | unchanged |
| 19 | Relative accelerator pedal | % | unchanged |
| 20 | Throttle position B | % | unchanged |
| 21 | Throttle position C | % | unchanged |
| 22 | Short fuel trim, bank 1 | 0.01% | / 100 → % |
| 23 | Long fuel trim, bank 1 | 0.01% | / 100 → % |
| 24 | Short fuel trim, bank 2 | 0.01% | / 100 → % |
| 25 | Long fuel trim, bank 2 | 0.01% | / 100 → % |
| 26 | Engine run time | s | unchanged |
| 27 | Ignition timing advance | 0.1 degree | / 10 → degrees |
| 28 | Demanded engine torque | % of reference torque | unchanged |
| 29 | Actual engine torque | % of reference torque | unchanged |
| 30 | Engine reference torque | Nm | unchanged |
| 31 | Fuel rail gauge pressure | kPa | unchanged |
| 32 | Boost pressure, derived | kPa | unchanged; may be negative |
| 33 | Fuel consumption, derived | mL/100 km | / 1000 → L/100 km |
| 34 | Actual torque, derived | Nm | unchanged; may be negative |

## Phone receiver contract

Keep one assembly buffer per connection, with its sequence, count, expected next
index, and payloads. BLE delivers this characteristic's notifications in order.

1. Reject headers shorter than five bytes, unknown versions, counts outside
   1–15, indexes outside the count, or payload lengths outside 1–239 bytes.
2. A fragment with index 0 starts an assembly. A newer sequence supersedes any
   partial assembly. Compare sequences with unsigned modular arithmetic:
   `distance = (incoming - previous) & 0xffff`; 1–32767 means newer. Ignore
   duplicates and older sequences. Clear this history on reconnect or after one
   second without any notification so a long outage or reboot can recover.
3. Subsequent fragments must match sequence/count and arrive at the expected
   index. Discard a malformed or interrupted assembly. Require equal nonfinal
   payload lengths and a final payload no longer than those lengths.
4. Decode only after receiving every fragment and verifying the combined length
   is 219 and the unused bitmap bits are zero. Replace the phone's entire live
   snapshot atomically; retain the sequence to reject duplicates.
5. Attach local monotonic receipt time to the complete snapshot. Store raw signed
   integers plus validity and sample ages. Convert units in the display layer.
   Between snapshots, display age as `wire_age_ms + local_elapsed_ms`; preserve
   the lower-bound meaning of a saturated age.
6. Mark the whole stream stale after one second without a **complete** snapshot,
   and mark it disconnected immediately on disconnect. New notifications do not
   make an old sample younger. An incomplete frame must not refresh UI values
   or the time of the last complete snapshot.

The controller uptime does not need to be synchronized to the phone. Use the
phone's monotonic receipt time for local freshness; retain controller capture
time if useful for diagnostics or later logging.

### Decoding example

This Python reference illustrates the same signed/little-endian decoding to use
with Swift byte reads or Kotlin `ByteBuffer.order(ByteOrder.LITTLE_ENDIAN)`:

```python
import struct


def decode_snapshot(body):
    if len(body) != 219 or body[8] & 0xf8:
        raise ValueError("Malformed version 1 snapshot")
    captured_ms, = struct.unpack_from("<I", body)
    readings = []
    for signal_id in range(35):
        value, age_ms = struct.unpack_from("<iH", body, 9 + 6 * signal_id)
        valid = bool(body[4 + signal_id // 8] & (1 << (signal_id % 8)))
        readings.append((value, age_ms) if valid else None)
    return captured_ms, readings
```

For an unfragmented notification beginning with `01 34 12 00 01`, the version is
1, sequence is 4660, and this is fragment 0 of 1. The following body starts at
notification byte 5. Body bytes `e8 03 00 00` mean capture time 1000 ms.
For any valid record, value bytes `d8 ff ff ff` mean -40; age bytes `32 00` mean
50 ms. Raw voltage 14500 displays as 14.5 V; fuel trim -781 displays as -7.81%.

## Build and verification

Use an activated ESP-IDF 6.1 environment. Existing local `sdkconfig` values take
precedence over `sdkconfig.defaults`; enable the defaults' BLE options in an
existing configuration. For a new checkout, select `esp32c6` with
`idf.py set-target esp32c6`, then run `idf.py build`.

A separate build can verify the tracked defaults without replacing local settings:

```sh
idf.py -B /tmp/brzi-ble-clean-build -DIDF_TARGET=esp32c6 \
  -DSDKCONFIG=/tmp/brzi-ble-clean-sdkconfig build
python3 tests/run_host_tests.py
```

The host suite compiles the production snapshot, encoder, and BLE lifecycle code
with address/undefined-behavior sanitizers and a simulated SDK. It checks stable
byte fixtures, ID ordering, one snapshot lock per tick, derived/invalid readings,
signed extrema, age saturation, timestamp/sequence rollover, all three MTUs,
subscription timing, reconnect/reset, failed connections, and allocation/send
failures followed by fresh data. It does not simulate the radio or phone OS.

The implementation passed the host suite and both existing-configuration and
fresh-default ESP32-C6 builds with ESP-IDF 6.1. A separate build of the original
sources with BLE disabled gives this comparison under the same SDK:

| Build metric | Before BLE | With BLE |
| --- | --- | --- |
| Firmware `.bin` size | 201,712 bytes | 607,568 bytes |
| Link-time DIRAM use, including code/data/BSS | 53,724 bytes | 94,677 bytes |
| Free space in the 1 MiB app partition | 81% | 42% |

The size increase includes the Bluetooth controller and NimBLE stack. Link-time
DIRAM use is not runtime free heap. No physical board/phone verification was
performed in this workspace.

Hardware acceptance remains required on both iOS and Android:

- Discover by service UUID and receive an immediate snapshot after subscribing.
- Observe approximately 10 complete snapshots/s at MTU 247 for several minutes;
  confirm smaller-MTU fragments assemble and gaps recover automatically.
- Unsubscribe, disconnect, reconnect, and reset the board; verify streaming and
  advertising resume correctly and old assemblies are cleared.
- Confirm unsupported/failed signals show unavailable, signed temperatures and
  fuel trims decode correctly, and derived readings match the serial monitor.
- Compare CAN update intervals and RX drops with BLE unsubscribed/subscribed.
  Record free/minimum heap and the NimBLE task's stack high-water mark while
  streaming; host tests and firmware size cannot establish runtime memory use.
