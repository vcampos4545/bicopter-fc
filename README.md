# bicopter-fc

A flight-control system for a bicopter: an ESP32 firmware target, a platform-independent
flight-control/estimation library, and a desktop physics simulator that runs the *identical*
flight-control code as the real vehicle.

This is real embedded flight software (ESP-IDF + FreeRTOS, native C/C++ peripheral APIs — not
Arduino), structured to stay readable for an engineer learning embedded flight software.

**Status:** Milestone 2 (ESP-IDF boots) is complete. `firmware/` is a real ESP-IDF v5.5.5
project targeting `esp32`; boot is verified in QEMU (see [docs/firmware.md](docs/firmware.md)).
No drivers, controllers, estimator, or simulator physics exist yet — see [TODO.md](TODO.md) for
the full roadmap and what's implemented so far.

## Target vehicle

Hardware is not yet finalized; every hardware-specific detail is meant to be configurable rather
than hardcoded (see [docs/hardware.md](docs/hardware.md)).

- ESP32 (specific board TBD)
- 2 brushless motors, each driven by a BLHeli ESC
- 2 tilt servos, one per motor (this is a bicopter: pitch/roll authority comes partly from
  differential thrust, partly from vectoring each motor's thrust angle via its servo)
- MPU6050 IMU(s)
- A barometric pressure sensor — BMP390 or BMP581; the estimator must not be coupled to a
  specific part (see [docs/estimation.md](docs/estimation.md))
- A LiPo battery with voltage/current monitoring
- A future radio link (ESP-NOW first, conventional RC receiver later)
- No GPS initially

## Architecture summary

The system is split into four independently-buildable pieces so that the flight-control logic
can be developed and tested on a desktop machine long before hardware exists, and so the same
estimator/controller code that flies the vehicle is the code exercised in simulation — not a
reimplementation of it.

```
firmware/     ESP-IDF project. FreeRTOS tasks, ESP32 peripheral drivers, and the glue that wires
              real sensors/actuators/radio into flight_core via the hardware-abstraction
              interfaces. This is the only piece that depends on ESP-IDF.

flight_core/  Platform-independent C++: state estimation, control (rate/attitude/allocation),
              vehicle dynamics model, math utilities, safety/failsafe logic. Depends ONLY on
              the hardware-abstraction interfaces (plain C++ types/enums), never on ESP-IDF or
              any desktop-only library. This is the code that actually flies the vehicle, and
              the code the simulator exercises.

simulator/    Desktop C++ program: implements the hardware-abstraction interfaces against a
              simulated rigid-body physics model instead of real hardware, links flight_core
              unmodified, and (later) visualizes the result. Lets the flight-control code be
              developed and validated without hardware.

tests/        Unit/integration tests, primarily against flight_core (and later, hardware
              abstraction implementations).
```

See [docs/architecture.md](docs/architecture.md) for the full pipeline (hardware → RTOS →
estimation → control → simulation → communications → safety), the hardware-abstraction
interface design that lets identical flight_core code run on both ESP32 and the desktop
simulator, and the FreeRTOS task list with target rates/priorities.

### Why one flight_core, two runtimes

`flight_core` never calls ESP-IDF APIs and never touches a specific sensor/actuator driver.
It only sees small abstract interfaces (`Imu`, `Barometer`, `MotorOutput`, `ServoOutput`,
`Radio`, ...). `firmware/` implements those interfaces against real ESP32 peripherals;
`simulator/` implements them against a physics model. Both link the same `flight_core` sources.
This is the mechanism that makes "test the flight controller in simulation" mean something —
the simulator is not a separate reimplementation that could drift from what flies.

## Coordinate frame and units convention

Used consistently across all code and documentation in this repository:

- **Units:** SI throughout the internal API — meters, seconds, radians, kilograms, Newtons,
  Newton-meters, Tesla, Pascals. Convert to/from sensor-native or protocol-native units
  (e.g. degrees, mG, raw ADC counts, PWM microseconds) only at the hardware-abstraction
  boundary, never inside `flight_core`.
- **Body frame:** right-handed, **X = forward, Y = right, Z = down** (standard aerospace body
  frame). Roll is rotation about X (positive = right wing down), pitch is rotation about Y
  (positive = nose up... i.e. positive pitch rate raises the nose per the right-hand rule about
  +Y being "right"; see [docs/control.md](docs/control.md) once the sign conventions are
  derived alongside the control-allocation math), yaw is rotation about Z (positive = nose
  right, clockwise viewed from above).
- **World frame:** NED (North-East-Down), matching the body-frame handedness so that at zero
  attitude the body frame is aligned with the world frame.
- **Attitude representation:** to be chosen in the state-estimator milestone
  ([docs/estimation.md](docs/estimation.md)); whatever is chosen must round-trip cleanly with
  the body-frame convention above.

## Build instructions

`firmware/` is buildable as of Milestone 2; the other components are not yet. This section
documents the intended build story per component; it will be filled in as each piece lands.

### firmware/ (ESP-IDF)

```sh
source ~/esp/esp-idf/export.sh   # after installing ESP-IDF v5.5.5, see docs/firmware.md
cd firmware
idf.py build
idf.py -p <PORT> flash monitor   # real hardware; not yet verified, see docs/firmware.md
idf.py qemu                      # boot verification without hardware
```

See [docs/firmware.md](docs/firmware.md) for the full toolchain install, build, and QEMU
boot-verification story.

### flight_core/ (platform-independent library)

Not yet implemented beyond directory scaffolding. Intended to build as a standalone static
library with a plain CMake project (no ESP-IDF dependency), consumable both by `firmware/`
(via ESP-IDF's CMake component system) and by `simulator/` (via a normal desktop CMake build).

```sh
cmake -S flight_core -B flight_core/build
cmake --build flight_core/build
```

### simulator/ (desktop physics simulator)

Not yet implemented beyond directory scaffolding and build-file stubs (Milestone 9: "bicopter
dynamics simulator" and Milestone 13: "simulator closed-loop stabilization").

```sh
cmake -S simulator -B simulator/build
cmake --build simulator/build
./simulator/build/bicopter_sim
```

### tests/

Not yet implemented. Intended to build against `flight_core` via CTest, independent of ESP-IDF
and the simulator.

## Documentation

- [TODO.md](TODO.md) — engineering roadmap, 18 milestones
- [docs/architecture.md](docs/architecture.md) — pipeline, data flow, HAL design, FreeRTOS tasks
- [docs/firmware.md](docs/firmware.md) — ESP-IDF toolchain version, build steps, QEMU boot verification
- [docs/hardware.md](docs/hardware.md) — hardware assumptions and how they're made configurable
- [docs/control.md](docs/control.md) — control approach (stub; derived in the control milestones)
- [docs/estimation.md](docs/estimation.md) — estimation approach (stub; derived in that milestone)
- [AGENTS.md](AGENTS.md) — structural/convention decisions for engineers and agents working on
  later milestones
