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

## Configuration philosophy

The rule this repository follows, stated once here so later milestones don't have to re-derive
it: **anything that changes if you swap a physical part or rewire a pin changes in exactly one
place**, and that place is always in `firmware/` (board/peripheral config) or in a vehicle-
parameters file consumed by both `flight_core/vehicle/` and `simulator/` (physical constants) —
never inside `flight_core/estimation/` or `flight_core/control/` logic itself, and never inside
`simulator/physics/`'s integration code. Those stay part-agnostic and vehicle-agnostic, reading
configured values rather than embedding them.
