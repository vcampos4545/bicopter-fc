# Hardware

Target hardware is **not yet finalized**. This document records every hardware assumption made
so far and how each is meant to be made configurable rather than hardcoded, so that later
milestones (and whoever finalizes the BOM) know exactly what's still open and where the
configuration seam belongs.

## Vehicle layout (fixed for now)

The vehicle *topology* is assumed fixed for this project regardless of exact part numbers:

- 2 brushless motors, each on its own BLHeli ESC
- 1 tilt servo per motor (thrust-vectoring bicopter, not a fixed-pitch quad-style layout)
- 1+ MPU6050 IMU
- 1 barometric pressure sensor
- 1 LiPo battery with voltage and current sensing
- 1 radio link (ESP-NOW initially, RC receiver later), no GPS

If the motor/servo count or arrangement ever changes, that's a bigger change than
"configuration" — it changes the control-allocation geometry (milestone 12) and the dynamics
model (milestone 9), not just a config value. Everything below this line is assumed
configurable without changing that topology.

## Open / TBD items

| Item | Status | Where it will be configured |
|---|---|---|
| ESP32 board/module (WROOM vs. WROVER vs. S3, pin mapping) | TBD | `firmware/`: a per-board config (e.g. Kconfig / a board header selected at build time) mapping logical peripherals (IMU bus, ESC PWM/DShot channels, servo PWM channels, battery ADC pins) to physical GPIOs. No GPIO numbers belong in `flight_core/` or `simulator/`. |
| ESC protocol (standard PWM, OneShot, DShot, ...) | TBD | Hidden entirely behind the `MotorOutput` interface (see [architecture.md](architecture.md#hardware-abstraction-layer)); `flight_core` only ever writes a normalized throttle in `[0, 1]`. Protocol selection is a `firmware/` build/runtime config, not a `flight_core` concern. |
| Servo type/range (analog PWM hobby servo vs. digital, angular range) | TBD | Hidden behind `ServoOutput`; `flight_core` writes an angle in radians in the vehicle's servo-frame convention, and the implementation maps that to the fitted servo's actual PWM range/center-trim, configured per-vehicle. |
| Barometer model (BMP390 vs. BMP581) | TBD | The `Barometer` interface (pressure + temperature) is part-agnostic; the specific register map/compensation algorithm lives entirely inside that one driver component in `firmware/components/sensors/`, selected at build time. `flight_core`'s altitude derivation from pressure must not assume either part's quirks. |
| Number of IMUs (single vs. redundant pair) | TBD, single assumed as the minimum case | Each IMU is one `Imu` instance; sensor-fusion-across-IMUs (if added) is `EstimatorTask` logic operating on however many `Imu` instances are configured, not a change to the interface. |
| Battery cell count / voltage-divider and current-sense scaling | TBD | Calibration constants (divider ratio, sense-resistor value or hall-sensor scale) live in `firmware/` configuration; the battery-monitor interface exposes calibrated volts/amps, not raw ADC counts. |
| Vehicle mass, inertia tensor, motor thrust/torque curves, servo tilt geometry (arm lengths, tilt-axis offsets) | TBD, needed for `simulator/physics/` and `flight_core/control/` allocation | These are physical constants of the *specific built vehicle*, not the firmware — they belong in a vehicle-parameters config (`flight_core/vehicle/`, per the directory layout) consumed by both the simulator's physics model and the control-allocation math, so both stay consistent with the same numbers. |
| RC receiver protocol (PPM/SBUS/CRSF/...) | Not started (milestone 15) | Behind a second `Radio`-shaped implementation, selected alongside/instead of ESP-NOW at build or runtime configuration. |
| IMU mounting orientation on the frame (which physical axis of the chip package faces "forward") | TBD until a board/frame is chosen | Any sign flip or axis permutation needed to align the MPU6050's package-marked axes with the body-frame convention (X=forward, Y=right, Z=down, see AGENTS.md) is a per-vehicle mounting config, applied where the driver's `imu_reading_t` is consumed — not hardcoded in `firmware/components/sensors/`. Milestone 3's driver reads the chip's native axes through as configured (range + calibration only); no axis remap is invented ahead of a real mounting decision. |

## MPU6050 (Milestone 3)

The IMU driver (`firmware/components/sensors/`) reads only raw accelerometer/gyroscope
registers — the MPU6050's DMP is never engaged, so state estimation stays an explicit
`flight_core/estimation/` concern in a later milestone rather than something baked into the
sensor.

| Item | Status | Where it will be configured |
|---|---|---|
| I2C address | 0x68 (AD0 strapped low) or 0x69 (AD0 strapped high); most breakout boards default to 0x68 | `mpu6050_config_t.i2c_address` (`firmware/components/sensors/include/mpu6050.h`), set by board-level init code once a board is chosen |
| SDA/SCL pins, I2C port/clock speed | TBD (no board chosen) | The I2C bus is created by the caller with `i2c_new_master_bus()` and passed into `mpu6050_init()` as `i2c_master_bus_handle_t`; this driver never picks pins itself |
| Data-ready (INT) pin | TBD (no board chosen); optional — see below | `mpu6050_config_t.data_ready_gpio`; pass `GPIO_NUM_NC` to disable and fall back to polling |
| Accel range, gyro range, DLPF bandwidth, sample-rate divider | Configurable, no fixed default assumed to be "correct" for the final vehicle | `mpu6050_config_t.convert` (`mpu6050_convert_config_t`) |
| Calibration offsets (accel bias in m/s^2, gyro bias in rad/s, per axis) | Configurable; determining actual bias values (e.g. a bench calibration routine) is out of scope for this milestone — see milestone 18 (bench-test tooling) | `mpu6050_convert_config_t.accel_offset_mps2` / `.gyro_offset_radps` |

### Interrupt-driven vs. polled reads

The driver uses the MPU6050's data-ready interrupt when a GPIO is configured, rather than
polling on a timer, so that `mpu6050_read()` (poll-style, matching the `Imu` HAL shape) only
performs an I2C burst read once a new sample genuinely exists — avoiding both wasted bus
transactions (polling faster than the configured sample rate) and added latency (polling slower
than it, and therefore reading stale-but-not-yet-overwritten registers). This also gives the
project a concrete, real place to demonstrate the ISR/task-context split described in
[architecture.md](architecture.md#interrupt-context-vs-task-context): the GPIO ISR
(`mpu6050_isr` in `firmware/components/sensors/src/mpu6050.c`) does nothing but capture
`esp_timer_get_time()` and hand that timestamp to task context via a length-1 FreeRTOS queue
(`xQueueOverwriteFromISR`) — no I2C transaction, no logging, no heap allocation happen in
interrupt context. The actual register read and unit conversion run in
`mpu6050_read()`, in whatever task calls it, which blocks on that queue (bounded by
`read_timeout_ms`) before doing the burst read.

`esp_timer_get_time()` (not `xTaskGetTickCount()`) is the timestamp source: it's a monotonic
microsecond-resolution counter safe to call from ISR context, and microsecond resolution matters
here since gyro integration error in the future estimator milestone is sensitive to sample
timing jitter — tick-count resolution (1ms by default) would be too coarse.

If `data_ready_gpio` is left unconfigured (`GPIO_NUM_NC`), `mpu6050_read()` instead polls the
`INT_STATUS` data-ready bit once per call — an ordinary I2C transaction, safe in task context —
before doing the burst read. This keeps the driver usable even before a board wires the INT pin
to a GPIO, at the cost of the latency/bus-utilization tradeoff above.

### What's verified vs. deferred

No physical MPU6050 was available in this environment. Verified: `idf.py build` succeeds with
this component included (clean full rebuild, no warnings from this project's code); the driver
was reviewed against the MPU6050 register map (RM-MPU-6000A-00) and product specification
(PS-MPU-6000A-00) for register addresses, bit meanings, and the documented power-on/wake
sequence; the calibration/unit-conversion/stale-detection logic has passing automated tests
(`tests/mpu6050_convert_test.c`, run via host `ctest` — see that file and
`tests/CMakeLists.txt`). Not verified: actual I2C transactions against real MPU6050 silicon,
real sensor readings, or the GPIO/ISR data-ready path on real hardware (or in QEMU — QEMU's
`esp32` machine does not model a real I2C peripheral with an attached MPU6050, so this was not
attempted). All of that is deferred until real hardware is available.

## Configuration philosophy

The rule this repository follows, stated once here so later milestones don't have to re-derive
it: **anything that changes if you swap a physical part or rewire a pin changes in exactly one
place**, and that place is always in `firmware/` (board/peripheral config) or in a vehicle-
parameters file consumed by both `flight_core/vehicle/` and `simulator/` (physical constants) —
never inside `flight_core/estimation/` or `flight_core/control/` logic itself, and never inside
`simulator/physics/`'s integration code. Those stay part-agnostic and vehicle-agnostic, reading
configured values rather than embedding them.
