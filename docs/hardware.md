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
| ESC protocol (standard PWM, OneShot, DShot, ...) | **Standard PWM implemented (Milestone 5); DShot/OneShot TBD** | Hidden entirely behind the `motor_output_t` interface (see [architecture.md](architecture.md#hardware-abstraction-layer) and [PWM ESC + tilt servo (Milestone 5)](#pwm-esc--tilt-servo-milestone-5) below); `flight_core` only ever writes a normalized throttle in `[0, 1]`. Which concrete implementation (`PwmEscOutput` today, a future `DshotEscOutput`) is a `firmware/` build/runtime config, not a `flight_core` concern. |
| Servo type/range (analog PWM hobby servo vs. digital, angular range) | **PWM output implemented (Milestone 5); exact servo part/range TBD** | Hidden behind `ServoOutput`; `flight_core` writes an angle in radians in the vehicle's servo-frame convention, and the implementation maps that to the fitted servo's actual PWM range/center-trim, configured per-vehicle. See [PWM ESC + tilt servo (Milestone 5)](#pwm-esc--tilt-servo-milestone-5) below. |
| Barometer model (BMP390 vs. BMP581) | **Chosen: BMP581 (Milestone 4)** | The `Barometer` interface (pressure + temperature + altitude) is part-agnostic; the specific register map lives entirely inside `firmware/components/sensors/bmp581.c`, selected at build time. See [BMP581 (Milestone 4)](#bmp581-milestone-4) below for why this part was picked over the BMP390 and what's configurable. |
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

## BMP581 (Milestone 4)

**BMP581 was picked over BMP390** for the reason the milestone brief asked to optimize for: a
more straightforward I2C register interface to implement well. The BMP390 (like the closely
related BME280/BMP388 family) outputs raw ADC counts that must be run through a per-device
polynomial compensation formula using calibration coefficients read from the chip's OTP memory at
startup — many magic constants, easy to transcribe wrong without the physical datasheet in hand.
The BMP581 instead outputs already-compensated pressure and temperature directly from its data
registers (no OTP trim-coefficient read or polynomial to apply), which is both a simpler driver
and less risk of a subtly-wrong compensation formula going unnoticed until real hardware exists to
check it against.

The driver (`firmware/components/sensors/bmp581.c`, `bmp581_convert.c`) reads only the
already-compensated `TEMP_DATA`/`PRESS_DATA` registers — there is no DMP-equivalent to avoid here
(unlike the MPU6050) since the BMP581 has no onboard fusion, just pressure/temperature.

| Item | Status | Where it will be configured |
|---|---|---|
| I2C address | 0x46 (SDO low) or 0x47 (SDO high); board TBD | `bmp581_config_t.i2c_address` (`firmware/components/sensors/include/bmp581.h`), set by board-level init code once a board is chosen |
| SDA/SCL pins, I2C port/clock speed | TBD (no board chosen) | The I2C bus is created by the caller with `i2c_new_master_bus()` and passed into `bmp581_init()` as `i2c_master_bus_handle_t`; this driver never picks pins itself, same convention as `mpu6050_init()` |
| Output data rate (ODR) | Configurable; defaults to the sensor's fastest rate | `bmp581_config_t.odr_reg_value` — a **raw passthrough** of the ODR_CONFIG register's 5-bit ODR field, not a named-rate enum (see below for why) |
| Sea-level reference pressure (for altitude) | Configurable, ISA standard (101325 Pa) is not assumed correct for the actual deployment field | `bmp581_convert_config_t.sea_level_pa` |
| Pressure smoothing | Configurable exponential-moving-average coefficient | `bmp581_convert_config_t.filter_alpha` — see below |

### Interface shape: pressure, temperature, and altitude together

`docs/architecture.md`'s Milestone-1 sketch of the `Barometer` interface described altitude
derivation as living in `flight_core`, with the driver exposing only pressure and temperature.
This milestone's brief explicitly asked for the driver to also expose a derived altitude (standard
barometric formula, configurable sea-level reference) and configurable filtering, so
`architecture.md`'s working sketch has been updated in this same change to match: `barometer.h`'s
`barometer_reading_t` now carries `pressure_pa`, `temperature_c`, `altitude_m`, `timestamp_us`, and
`valid`. This is not a contradiction of the fusion work planned for the state estimator (Milestone
8, per `TODO.md`): the driver's `altitude_m` is a simple, single-sensor barometric-formula estimate
useful on its own (e.g. telemetry/display, or as a coarse baseline), while the estimator's future
altitude *state estimate* fuses this pressure data with IMU vertical acceleration for a
better-behaved (lower-latency, less noisy) result. `flight_core`'s estimator is free to consume
either `altitude_m` or the raw `pressure_pa` field, whichever fits its fusion approach once
Milestone 8 derives it.

The interface also deviates from `architecture.md`'s original Kelvin sketch: `temperature_c` is
Celsius, not Kelvin, because the milestone brief asked for it explicitly and Celsius is what's
directly useful for telemetry/display; the "convert at the hardware-abstraction boundary" principle
in `AGENTS.md` is satisfied either way, since the boundary is exactly where this unit decision is
made explicit. A future strict-SI consumer inside `flight_core` can add 273.15 at its own call
site if it ever needs Kelvin specifically.

### No data-ready interrupt

Unlike the MPU6050 driver, `bmp581_read()` does not use a data-ready interrupt. The sensor is
placed into continuous (free-running) power mode at `bmp581_init()` time and keeps sampling at its
configured ODR in the background; `bmp581_read()` simply performs one I2C burst read of whatever is
currently in the `TEMP_DATA`/`PRESS_DATA` registers. This is a deliberate scope simplification, not
an oversight: per `docs/architecture.md`'s FreeRTOS task table, baro dynamics are far slower than
attitude dynamics (~50 Hz vs. 500 Hz–1 kHz), so the latency/jitter an interrupt-driven handoff
buys the MPU6050 driver isn't worth the added ISR complexity here. If a future milestone finds
otherwise on real hardware, the same ISR/queue pattern `mpu6050.c` uses is the template to follow.

### Pressure filtering

`bmp581_convert_config_t.filter_alpha` configures a single-pole exponential moving average (EMA)
applied to pressure before altitude conversion: `filtered = alpha*raw + (1-alpha)*prev`, seeded
directly from the first sample (no ramp-up from zero). `alpha = 1.0` disables smoothing entirely;
lower values trade response latency for noise rejection. A simple EMA was chosen over a
moving-average window because it needs only one `float` of state per sensor instance (no sample
history buffer) and is the simplest filter shape that is still meaningfully configurable, matching
this milestone's "your call on exact filter shape, but document it" instruction. The filter math
lives in `bmp581_convert.c` (`bmp581_filter_apply`) and is host-tested in
`tests/bmp581_convert_test.c`.

### Why the ODR register field is a raw passthrough, not a named-rate enum

The MPU6050 driver's `mpu6050_dlpf_bandwidth_t` enum maps named bandwidths to register bits
because that table is small (7 entries) and well cross-referenced in this driver author's
training data. The BMP581's ODR field is 5 bits (32 possible rates spanning 240 Hz down to
0.125 Hz per the datasheet's ODR table), and this driver's author has lower confidence
transcribing that full table correctly from memory without the physical datasheet PDF in hand —
getting an intermediate entry wrong would be a silently-wrong sample rate, not a build failure.
Rather than risk that, `bmp581_config_t.odr_reg_value` is a raw `uint8_t` passthrough of the
ODR_CONFIG register's ODR[6:2] field; `BMP581_ODR_FASTEST` (0x00, the sensor's power-on-reset
default) is the only value asserted with confidence and is a safe default that doesn't depend on
the rest of the table being right. Whoever brings up real hardware should cross-check the full ODR
table against the physical BMP581 datasheet (document BST-BMP581-DS004) and can pass any other raw
register value directly once confirmed — see `bmp581_registers.h`.

### What's verified vs. deferred

No physical BMP581 was available in this environment. Verified: `idf.py build` succeeds with this
component included (clean full rebuild, no warnings from this project's code); pressure/temperature
register scaling, EMA filtering, barometric-formula altitude derivation, and stale/invalid-data
detection have passing automated tests (`tests/bmp581_convert_test.c`, run via host `ctest`).
Not verified, and held to a *lower* confidence bar than Milestone 3's MPU6050 driver: the register
addresses and bit layout themselves (`bmp581_registers.h`) were transcribed from this driver
author's recollection of the BMP581 datasheet without a physical datasheet PDF or physical chip
present in this environment to check against — unlike the MPU6050, whose register map is far more
widely cross-referenced. Also deferred, same as Milestone 3: actual I2C transactions against real
BMP581 silicon, and real pressure/temperature readings. All of this should be re-verified against
the physical BMP581 datasheet and real hardware before flight.

## PWM ESC + tilt servo (Milestone 5)

The actuator driver (`firmware/components/actuators/`) drives both BLHeli ESCs and both tilt
servos with conventional RC PWM generated on the ESP32's LEDC peripheral. No physical ESC, servo,
or motor was available in this environment, and this milestone explicitly never arms anything -
see What's verified vs. deferred at the end of this section.

### Why LEDC over MCPWM

ESP-IDF offers two peripherals that can generate this kind of signal: LEDC and MCPWM. LEDC was
picked because everything this milestone needs is a fixed-frequency, independently-adjustable
duty cycle per channel (4 channels total: 2 ESC + 2 servo), which is exactly LEDC's use case and
API shape (`ledc_timer_config()` + `ledc_channel_config()` + `ledc_set_duty_and_update()`).
MCPWM exists for motor-control signal patterns this project doesn't need yet - complementary
outputs with configurable dead-time, synchronized multi-channel phase relationships, fault/trip
inputs - and would be a heavier API for the same result. If a future milestone needs one of those
(e.g. a brushed-motor H-bridge, which this project does not use), that's the point to bring in
MCPWM alongside LEDC, not to replace it.

### Interface shapes

- **`ServoOutput`** (`servo_output.h`) is a plain opaque-handle driver, the same shape as
  `mpu6050.h`/`bmp581.h`: one implementation (PWM) is all that's needed, since hobby/digital tilt
  servos are overwhelmingly PWM-controlled and no alternative protocol is in view.
- **`MotorOutput`** (`motor_output.h`) is instead a small C "vtable" - an ops struct of
  `write`/`set_idle`/`deinit` function pointers plus an opaque `ctx` - because the brief explicitly
  anticipates a second, non-PWM implementation (DShot). `pwm_esc_output.c/.h` populates that
  vtable via `pwm_esc_output_as_motor_output()`; a future `dshot_esc_output.c/.h` would do the
  same from its own state. Code that only ever holds a `motor_output_t` (this milestone's
  `actuators_init_safe()`, and later milestone 12's control allocation) does not change when that
  lands - this is the "clean addition, not a rewrite" seam the milestone brief asked for. DShot
  itself is **not implemented** here; this is only the extension point.

### ESC protocol: how to determine what your BLHeli ESCs actually support

This project assumes standard RC PWM (a 1000-2000us pulse, repeated at a configurable frequency)
because it is universally supported by every BLHeli/BLHeli_S/BLHeli32 ESC ever made - it's the
lowest common denominator. Before assuming a specific board's ESCs support something faster
(DShot150/300/600, Multishot, OneShot125), check:

1. **The ESC's firmware and version.** Plain BLHeli (not BLHeli_S or BLHeli32) generally does not
   support DShot. BLHeli_S and BLHeli32 do. Reading the firmware off the ESC requires BLHeli
   Suite (Windows/Linux GUI tool) connected via a compatible USB-to-serial passthrough
   (a cheap "BLHeli passthrough" adapter, or a flight controller already wired to the ESCs'
   signal wires running passthrough firmware) - there is no way to determine this from the ESC's
   markings alone in general, though many modern ESCs print their BLHeli variant on the case.
2. **The seller/datasheet listing**, which for a modern ESC usually states the supported
   protocols directly (e.g. "supports DShot600/300/150, Multishot, Oneshot125, PWM").
3. **If in doubt, use PWM.** It costs nothing but the (real, but for this vehicle's control rates
   not disqualifying) throughput/latency headroom a digital protocol would buy - see
   docs/architecture.md's FreeRTOS task table: `FlightControlTask` targets 250-500Hz, well within
   what even 50Hz PWM's ~2ms worst-case actuation latency can serve, since ESC spin-up/servo slew
   time already dominates that latency budget far more than the command protocol does.

`pwm_esc_output_config_t.freq_hz` is configurable specifically so a board with confirmed
higher-PWM-rate-tolerant ESCs (many BLHeli_S/32 ESCs accept PWM well above the traditional 50Hz,
sometimes 400Hz+) can raise it once that's confirmed for the actual parts in hand - do not raise
it blind; an ESC that doesn't tolerate a shorter period can behave unpredictably.

### ESC calibration / endpoint-setting, and what it means for `min_throttle`/`max_throttle`

Nearly every PWM-driven ESC (BLHeli included) has a **learned throttle range**: on power-up, it
compares the very first PWM pulse width it sees against its stored high/low endpoints to decide
"is this a calibration signal or a normal throttle command." The standard calibration procedure
(consult your specific ESC's documentation for exact button/beep sequences, since BLHeli variants
differ) is:

1. Disconnect the ESC's battery power. Set the transmitter/signal source to maximum throttle.
2. Power on the ESC while it's receiving the max-throttle signal; it beeps to confirm the high
   point is learned.
3. Immediately move the signal to minimum throttle; the ESC beeps again to confirm the low point,
   then arms normally from then on.

Once calibrated, the ESC's own learned endpoints - not necessarily exactly 1000us/2000us - are
what a given pulse width means to it. `pwm_esc_convert_config_t.min_pulse_us`/`max_pulse_us`
should be set to match whatever the ESC was actually calibrated to (or, more simply, calibrate
every ESC on the vehicle to exactly 1000us/2000us using the procedure above, and use those
literal values here - the more common and less error-prone approach). This driver does **not**
implement an interactive calibration routine itself (out of scope for this milestone, and safer
performed deliberately, off-vehicle, with props removed, per standard practice) - milestone 18
(bench-test tooling) is the natural place for a guided calibration flow if one is wanted. What
this driver gives instead: `min_throttle`/`max_throttle` (a normalized sub-range clamp, separate
from the raw pulse endpoints) so a caller can restrict the *commandable* range below what the ESC
is calibrated to, without re-calibrating the ESC itself - useful for a conservative first-power-on
throttle ceiling, for instance.

### `invert_direction`: what it does and doesn't do

`pwm_esc_convert_config_t.invert_direction` swaps which pulse endpoint throttle `0.0`/`1.0` map
to. Be clear about what this can and cannot fix: it does **not** reverse a conventional
(non-bidirectional) BLHeli ESC's physical motor rotation direction - that is set either by
physically swapping any two of the three motor phase wires, or via a "motor direction" parameter
in the ESC's own firmware configuration (BLHeli Suite), and is out of this project's scope to
automate. `invert_direction` exists for wiring/protocol quirks where the *sense* of the throttle
command itself needs flipping (e.g. a bidirectional/"3D mode" ESC where the pulse-width range is
centered and throttle direction is signal-relative) - document on a per-unit basis if and when a
real vehicle build needs it; do not reach for it as a substitute for the ESC-level fix above.

`servo_convert_config_t.invert_direction` is the more straightforwardly useful case: a tilt
servo mounted with its horn on the opposite side from its counterpart unit needs its angle
command mirrored so "more positive tilt" means the same physical direction on both units - this
is a normal, expected per-unit mounting config, not a workaround.

### Tilt-servo geometry assumptions

No servo part or vehicle frame is chosen yet, so nothing here assumes a specific mechanical tilt
range. `servo_convert_config_t` carries `min_angle_rad`/`max_angle_rad`/`neutral_angle_rad` as
plain configuration - the milestone brief's example of a tilt-rotor bicopter typically wants
something on the order of tens of degrees of tilt authority per side, but the actual number
depends on the frame/motor-mount geometry chosen when the vehicle is built, and is a
`firmware/` board-config value, never hardcoded in `servo_output.c`/`servo_convert.c`. "Radians,
servo-frame" per docs/architecture.md's `ServoOutput` sketch means the angle is in this driver's
own configured frame (whatever `min_angle_rad`/`max_angle_rad`/`neutral_angle_rad` define it to
be) - milestone 12's control allocation is responsible for translating body-frame tilt demand
into this frame once the actual mechanical relationship between servo angle and thrust-vector
angle is known.

### Safety / init behavior

Per the top-level design brief's hard requirement: on startup, motors must not be armed/active
and servos must move to a safe neutral position before anything else touches actuators. This is
implemented at two levels, not just documented as a convention:

1. **Each unit's own `_init()` already idles/neutrals as a side effect of construction.**
   `pwm_esc_output_init()` configures its LEDC channel with `convert.idle_pulse_us` as the
   *initial* duty (never an undefined or zero duty, which a BLHeli ESC could read as "no signal"
   rather than a deliberate idle command); `servo_output_init()` does the same with
   `convert.neutral_angle_rad`. There is no window, even for a single unit, where a handle exists
   but hasn't been idled/neutraled yet.
2. **`actuators_init_safe()`** (`actuators_init.h`) is the single sanctioned entry point that
   produces working actuator handles at all - it initializes both tilt-rotor units in order, and
   on any single unit's init failure, deinitializes everything already created before returning
   the error, so a caller can never be handed a partially-initialized `actuators_t`. This function
   is not wired into `main/` yet - that is milestone 6's task-architecture job - but it exists now
   as the obvious, explicit code path that milestone will call before starting any other actuator
   activity.

Out-of-range commands (`pwm_esc_output_write()`/`servo_output_write()` given a throttle/angle
outside the configured range, including non-finite input) are **clamped and logged** (`ESP_LOGW`),
not rejected outright. This was chosen over hard rejection because a flight-control loop should
never leave an actuator holding a stale or uncommanded output just because one input sample was
out of range - always producing a bounded, defined output is the safer behavior for a real-time
control path, while the warning log still makes the out-of-range condition visible for
post-flight diagnosis. The pure clamping logic itself (`pwm_esc_clamp_throttle()`/
`servo_clamp_angle()`) is what's unit-tested; the logging call sites are the thin ESP-IDF-facing
wrappers around it.

### What's verified vs. deferred

No physical ESC, servo, or motor was available in this environment, and per this milestone's
explicit scope, nothing was armed. Verified: `idf.py build` succeeds with this component included
(clean full rebuild, no warnings from this project's code); the normalized-throttle/angle clamping,
throttle/angle-to-pulse-width mapping, and pulse-width-to-LEDC-duty-count math have passing
automated tests (`tests/pwm_esc_convert_test.c`, `tests/servo_convert_test.c`,
`tests/pwm_util_test.c`, run via host `ctest`); the LEDC configuration code
(`pwm_esc_output.c`/`servo_output.c`) was reviewed against ESP-IDF's `driver/ledc.h` API (read
directly from the installed ESP-IDF v5.5.5 source tree, not from memory) and against standard RC
PWM/BLHeli timing conventions (1000-2000us pulse, configurable frequency, ESC-learned endpoint
calibration). Not verified, and deferred until real hardware is available: actual PWM signal
generation on a real GPIO/oscilloscope, whether a real ESC/servo responds correctly to the
generated signal, and the interactive ESC-calibration procedure described above (which was not
implemented, only documented).

## Configuration philosophy

The rule this repository follows, stated once here so later milestones don't have to re-derive
it: **anything that changes if you swap a physical part or rewire a pin changes in exactly one
place**, and that place is always in `firmware/` (board/peripheral config) or in a vehicle-
parameters file consumed by both `flight_core/vehicle/` and `simulator/` (physical constants) —
never inside `flight_core/estimation/` or `flight_core/control/` logic itself, and never inside
`simulator/physics/`'s integration code. Those stay part-agnostic and vehicle-agnostic, reading
configured values rather than embedding them.
