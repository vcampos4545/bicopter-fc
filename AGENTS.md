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
battery monitor (exact shape TBD when the ESC/servo HAL milestone lands). `firmware/`
implements each against real ESP32 peripherals; `simulator/` implements each against a physics
model. Both link the same `flight_core` sources unmodified.

Rule of thumb for "does this belong in `flight_core` or behind the HAL": if it changes when you
swap a physical part (different IMU, different ESC protocol, different servo), it's a `firmware/`
config or a HAL implementation detail, not `flight_core` logic. Full interface sketches and the
reasoning: [docs/architecture.md](docs/architecture.md#hardware-abstraction-layer). Open
hardware questions and where each is meant to become configurable:
[docs/hardware.md](docs/hardware.md).

## What's real vs. stub right now

As of Milestone 2: `firmware/` is a real, buildable ESP-IDF project (see below) with a minimal
`main/` that boots FreeRTOS and logs — no sensor/actuator/radio/estimation/control code yet.
`flight_core/CMakeLists.txt` and `simulator/CMakeLists.txt` are still non-functional placeholders
(commented-out build wiring) — do not expect either to configure or build yet.
`docs/control.md` and `docs/estimation.md` are intentionally stubs: the force/torque model,
control-allocation math, and attitude-estimation algorithm must be *derived* from real vehicle
geometry/sensor data in their respective milestones, not invented ahead of time. See
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
