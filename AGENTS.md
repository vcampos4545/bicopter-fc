# AGENTS.md

Structural and convention decisions made in Milestone 1 that later milestones must stay
consistent with. This file records decisions, not derivations — see the linked docs for the
reasoning.

## Directory layout

```
firmware/     ESP-IDF project (ESP32, C/C++, native peripheral APIs — not Arduino).
              main/, components/{sensors,actuators,radio,estimation,control,safety,telemetry}/
flight_core/  Platform-independent C++. estimation/, control/, dynamics/, math/, vehicle/,
              safety/. Depends ONLY on the hardware-abstraction interfaces — never on ESP-IDF
              or any desktop-only library. This is the code that flies the vehicle AND the code
              the simulator runs; do not fork it or reimplement it per-target.
simulator/    Desktop C++ simulator. physics/, sensors/, visualization/, main.cpp. Links
              flight_core unmodified against simulated hardware-abstraction implementations.
tests/        Tests, primarily against flight_core.
docs/         architecture.md, hardware.md, control.md, estimation.md.
```

Empty scaffold directories currently contain a `.gitkeep`; remove it in the same commit that
adds the first real file there.

Full rationale: [docs/architecture.md](docs/architecture.md).

## Body-frame / units convention — do not change without updating every doc

- SI units throughout `flight_core`'s internal API (meters, seconds, radians, kg, N, N·m, Pa,
  Tesla). Sensor-native or protocol-native units (degrees, mG, PWM microseconds, raw ADC counts)
  convert at the hardware-abstraction boundary, never inside `flight_core`.
- Body frame: right-handed, **X = forward, Y = right, Z = down**. World frame: NED. Roll about
  X, pitch about Y, yaw about Z.
- This convention is stated in [README.md](README.md) and referenced from every doc that touches
  attitude/control math. If it ever needs to change, update it in all of them in the same
  change, not just the file you're editing.

## Hardware abstraction layer (HAL)

`flight_core` never calls ESP-IDF APIs or touches a specific sensor/actuator part. It only sees
small poll-style interfaces — `Imu`, `Barometer`, `MotorOutput`, `ServoOutput`, `Radio`, and a
battery monitor (exact shape still TBD; Milestone 5 landed `MotorOutput`/`ServoOutput` but did not
address battery sensing — that remains open for whichever milestone first needs it). `firmware/`
implements each against real ESP32 peripherals; `simulator/` implements each against a physics
model. Both link the same `flight_core` sources unmodified.

Rule of thumb for "does this belong in `flight_core` or behind the HAL": if it changes when you
swap a physical part (different IMU, different ESC protocol, different servo), it's a `firmware/`
config or a HAL implementation detail, not `flight_core` logic. Full interface sketches and the
reasoning: [docs/architecture.md](docs/architecture.md#hardware-abstraction-layer). Open
hardware questions and where each is meant to become configurable:
[docs/hardware.md](docs/hardware.md).

Until `flight_core` exists, each `firmware/components/*` driver defines its own local plain-C
struct matching its interface's documented shape (e.g. `firmware/components/sensors/include/
imu.h`'s `imu_reading_t`, added in Milestone 3) instead of waiting on a not-yet-real shared
header. These are deliberately free of ESP-IDF types so they double as the boundary pure/testable
logic is written against (see below). When `flight_core` adds the authoritative HAL interface
headers, reconcile each driver's local copy against them in that same change rather than leaving
two shapes to drift apart.

## Driver testing convention

Firmware-side I/O (I2C/SPI transactions, GPIO/ISR behavior) can't be unit tested without real
silicon or a peripheral-accurate emulator (QEMU's `esp32` machine does not model attached I2C
devices). The convention, established in Milestone 3 for the MPU6050 driver: split each driver
into (a) hardware I/O code that depends on ESP-IDF types and is reviewed against the part's
datasheet but not automated-tested pre-hardware, and (b) pure, ESP-IDF-free logic (register
scaling, calibration, staleness/validity checks) that *is* automated-tested, by compiling it
directly (via a relative source path, no `flight_core` dependency yet) into a standalone host
executable under `tests/`, built with plain CMake/CTest independent of ESP-IDF — see
`tests/CMakeLists.txt` and `tests/mpu6050_convert_test.c`. Milestone 4 applied the same split to
the BMP581 barometer driver (`tests/bmp581_convert_test.c`), and Milestone 5 to the ESC/servo PWM
output drivers (`tests/pwm_esc_convert_test.c`, `tests/servo_convert_test.c`,
`tests/pwm_util_test.c`); apply it to later actuator/sensor drivers the same way.

## What's real vs. stub right now

As of Milestone 5: `firmware/` is a real, buildable ESP-IDF project (see below) with a minimal
`main/` that boots FreeRTOS and logs, real MPU6050 IMU and BMP581 barometer drivers in
`firmware/components/sensors/` (I2C init/config, interrupt-or-polled IMU reads / polled barometer
reads, raw-to-SI conversion with calibration and filtering, stale-data detection), and real
PWM ESC/servo output drivers in `firmware/components/actuators/` (LEDC-generated conventional RC
PWM behind the `motor_output_t`/`ServoOutput` interfaces, with an explicit `actuators_init_safe()`
safe-init entry point). None of these are yet wired into `main/` (that's milestone 6's task
architecture) and none is yet verified against real hardware (no physical MPU6050, BMP581, ESC,
servo, or motor was available; see [docs/hardware.md](docs/hardware.md#mpu6050-milestone-3),
[docs/hardware.md](docs/hardware.md#bmp581-milestone-4), and
[docs/hardware.md](docs/hardware.md#pwm-esc--tilt-servo-milestone-5)). No radio/estimation/control
code exists yet. `flight_core/CMakeLists.txt` and `simulator/CMakeLists.txt` are still
non-functional placeholders (commented-out build wiring) — do not expect either to configure or
build yet. `docs/control.md` and `docs/estimation.md` are intentionally stubs: the force/torque
model, control-allocation math, and attitude-estimation algorithm must be *derived* from real
vehicle geometry/sensor data in their respective milestones, not invented ahead of time. See
[TODO.md](TODO.md) for the full milestone sequence and current status.

## Firmware toolchain

- ESP-IDF **v5.5.5**, target chip **`esp32`** (pinned in `firmware/sdkconfig.defaults`; no
  specific board/pinout chosen yet — see [docs/hardware.md](docs/hardware.md)).
- Install/build/flash instructions and how boot is verified (QEMU, since no physical board is
  available in this environment): [docs/firmware.md](docs/firmware.md).
- `firmware/sdkconfig` (the full generated config) is gitignored; only `sdkconfig.defaults` is
  committed. A fresh checkout just needs `idf.py build` — target is already pinned.

## Maintaining this file

Keep this file proportionate: only record decisions that constrain *future* work across
milestones (conventions, interface shapes, layout) — not implementation narration, not anything
`git log`, the code itself, or the docs it links to already show authoritatively. When a later
milestone changes something recorded here (e.g. extends a HAL interface, revises a task rate),
update the relevant entry in the same change rather than leaving this file stale.
