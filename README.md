# BRZI

ESP32-C6 OBD2 dashboard data collector, using 11-bit classic CAN and Mode 01.

## Live signals

The scheduler supports the following 32 PIDs. Intervals are minimum waits after
completion; actual refresh rates depend on ECU response time and other jobs.

| PID (hex) | Signal | Stored unit | Interval |
| --- | --- | --- | --- |
| 04 | Calculated engine load | % | 1 s |
| 05 | Coolant temperature | C, signed | 1 s |
| 06 / 08 | Short-term fuel trim, banks 1 / 2 | 0.01% | 1 s |
| 07 / 09 | Long-term fuel trim, banks 1 / 2 | 0.01% | 5 s |
| 0B | Intake manifold absolute pressure | kPa | 500 ms |
| 0C | Engine RPM | rpm | 100 ms |
| 0D | Vehicle speed | km/h | 200 ms |
| 0E | Ignition timing advance | 0.1 degree | 1 s |
| 0F | Intake air temperature | C, signed | 2 s |
| 10 | Mass airflow | mg/s | 1 s |
| 11 | Throttle position | % | 500 ms |
| 1F | Engine run time | s | 5 s |
| 23 | Fuel rail gauge pressure | kPa | 1 s |
| 2F | Fuel tank level | % | 5 s |
| 33 | Barometric pressure | kPa | 5 s |
| 42 | Control module supply voltage | mV | 2 s |
| 45 | Relative throttle position | % | 1 s |
| 46 | Ambient air temperature | C, signed | 5 s |
| 47 / 48 | Throttle position sensors B / C | % | 1 s |
| 49 | Accelerator pedal position D | % | 250 ms |
| 4A / 4B | Accelerator pedal position E / F | % | 1 s |
| 4C | Commanded throttle actuator | % | 1 s |
| 5A | Relative accelerator pedal position | % | 250 ms |
| 5C | Engine oil temperature | C, signed | 1 s |
| 5E | Engine fuel rate | mL/h | 1 s |
| 61 / 62 | Demanded / actual engine percent torque | % of reference torque | 1 s |
| 63 | Engine reference torque | Nm | 30 s |

Pedal D/E/F identify accelerator sensor channels, which may be redundant. They
are not brake/clutch pedal positions, and absolute pedal readings need not span
exactly 0–100% of physical travel. Availability varies by vehicle.

All stored values remain integers. For example, `MODULE_VOLTAGE=14500 mV` is
14.5 V, `MAF=12340 mg/s` is 12.34 g/s, and `STFT_B1=-781 0.01%` is -7.81%.
Percentage signals other than fuel trims use whole percent, rounded toward zero.
The existing `OBD_SIGNAL_BATTERY_VOLTAGE` enum represents ECU supply voltage,
not battery state of charge.

## Capability detection and logging

On startup, support pages `00`, `20`, `40`, and `60` are queried. Bitmaps are kept
separately for responders `7E8`–`7EF`. Signals confirmed unsupported are skipped.
A missing response leaves support unknown, so the signal can still be tried;
incomplete capability discovery retries every ten seconds. Failed live queries
invalidate that reading and back off before retrying.

The serial monitor prints enabled signals, integer units, and reading ages every
second. `N/A` means a reading is not currently valid. Under load the scheduler
services the oldest overdue request first, with priority breaking ties.

Access values through `car_data_get()`, `car_data_get_full()`, or
`car_data_get_snapshot()`. The snapshot exposes named fields under `car.engine`.
Discovery and OBD queries share one requester and must run from the same task.

## Derived values

These fields are computed when car data is read, using valid, recent inputs:

- `boost_pressure_kpa`: manifold absolute pressure minus barometric pressure.
  Negative values indicate manifold vacuum. PID `0B` tops out at 255 kPa absolute,
  so its range may not cover high boost applications.
- `fuel_consumption_ml_100km`: `fuel_rate_ml_h * 100 / speed_kph`. Divide by 1000
  for L/100 km. It is invalid at standstill; fuel rate in L/h remains available.
- `actual_torque_nm`: actual torque percentage times reference torque, divided
  by 100. This uses the ECU's torque estimate, not an independent measurement.

No derived value is fabricated when its input PIDs are unavailable.

## BLE live telemetry

The device advertises as `BRZI` and accepts one phone without pairing. Subscribe
to its telemetry characteristic to receive all 35 readings immediately, then every
100 ms. Values stay in the controller's integer units and include validity and
sample age. At ATT MTU 247, each complete snapshot uses one 224-byte notification;
smaller MTUs use fragments. Streaming runs on NimBLE's host event queue and does
not add another application task or change OBD polling intervals.

See [the BLE protocol](docs/ble-protocol.md) for UUIDs, byte layout, signal IDs,
phone decoding/reassembly rules, build setup, and hardware acceptance checks.
`sdkconfig.defaults` enables the peripheral; existing local configurations must
also enable those settings because saved `sdkconfig` values take precedence.

## Verification

Run the host tests with a C compiler and Python 3:

```sh
python3 tests/run_host_tests.py
```

The tests compile the production code with a simulated SDK and use address and
undefined-behavior sanitizers. They cover decoding, units and signed values,
derived calculations, multiple ECU support bitmaps, discovery retries, and a
40-second scheduler simulation. BLE tests cover byte fixtures, fragmentation,
subscription/reconnect/reset behavior, and congestion recovery. They do not
replace testing on the car and physical iOS/Android phones.

Build firmware from an activated ESP-IDF 6.1 environment using `idf.py build`.
For a new checkout, run `idf.py set-target esp32c6` first.

PID references: [CSS Electronics](https://www.csselectronics.com/pages/obd2-pid-table-on-board-diagnostics-j1979)
and the [python-OBD command tables](https://python-obd.readthedocs.io/en/latest/Command%20Tables/).
