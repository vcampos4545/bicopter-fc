# bicopter-fc

A flight-control system for a bicopter: an ESP32 firmware target, a platform-independent
flight-control/estimation library, and a desktop physics simulator that runs the *identical*
flight-control code as the real vehicle.

This is real embedded flight software (ESP-IDF + FreeRTOS, native C/C++ peripheral APIs — not
Arduino), structured to stay readable for an engineer learning embedded flight software.

**Status:** Milestone 11 (attitude controller) is complete. `firmware/` is a real ESP-IDF
v5.5.5 project targeting `esp32`; boot is verified in QEMU (see
[docs/firmware.md](docs/firmware.md)), `firmware/components/sensors/` has real MPU6050 IMU and
BMP581 barometer I2C drivers, `firmware/components/actuators/` has real ESC/servo PWM output
drivers, and `firmware/main/` runs six FreeRTOS tasks (SensorTask, EstimatorTask,
FlightControlTask, RadioTask, TelemetryTask, SafetyTask) at documented priorities/periods with a
queue, a task notification, an `esp_timer`, a mutex, and the task watchdog wired between them —
task bodies are still stub/no-op logic (see [docs/architecture.md](docs/architecture.md)).
`flight_core/math/` is a real, unit-tested vector/matrix/quaternion library (see
[docs/math.md](docs/math.md)), `flight_core/estimation/` has a real, unit-tested attitude
estimator — a Mahony-style nonlinear complementary filter fusing gyro and accelerometer data into
a quaternion attitude + angular velocity, behind an interface a future EKF could implement as a
drop-in replacement (see [docs/estimation.md](docs/estimation.md)) — and `flight_core/vehicle/`
now holds `VehicleParams`, the shared vehicle-constants config (mass, inertia, motor/servo
geometry, thrust/torque/drag coefficients) meant to be consumed by both the simulator and the
future control-allocation milestone. `simulator/physics/` is a real, unit-tested rigid-body
forward-dynamics model (`m*v_dot=F` and `I*omega_dot + omega x (I*omega)=tau`, integrated with
semi-implicit Euler and Milestone 7's `Quaternion::integrate()`) — see
[docs/dynamics.md](docs/dynamics.md). `flight_core/control/` now holds a generic `Pid` building
block (configurable gains, output saturation, clamping anti-windup, a documented
derivative-on-measurement/derivative-on-error choice, explicit per-call `dt`) and a
`RateController` built on it — three independent roll/pitch/yaw-rate PID loops consuming
`AttitudeEstimate::angular_velocity_radps` and producing a desired body torque. As of Milestone
11, `flight_core/control/` also holds `AttitudeController` — a stateless outer loop converting a
desired/current attitude quaternion pair into a desired body rate via quaternion feedback
(`q_error = current.inverse() * desired`, a per-axis proportional gain on the error's small-angle
vector part, a double-cover shortest-path fix, and an always-on per-axis rate-limit clamp),
feeding `RateController`'s `desired_radps` input unchanged and completing the cascaded
attitude-loop-then-rate-loop architecture on paper — see [docs/control.md](docs/control.md) for
the full derivation. Nothing in `firmware/`/`simulator/` calls into `flight_core/estimation/` or
`flight_core/control/` yet — wiring the estimator into `EstimatorTask` is a noted follow-up (see
docs/estimation.md) — and control-allocation code remains unimplemented (Milestone 12). No real
hardware was available to verify actual I2C/PWM transactions or real-hardware task timing — see
[TODO.md](TODO.md) for the full roadmap and what's implemented so far.

## Target vehicle

Hardware is not yet finalized; every hardware-specific detail is meant to be configurable rather
than hardcoded (see [docs/hardware.md](docs/hardware.md)).

- ESP32 (specific board TBD)
- 2 brushless motors, each driven by a BLHeli ESC
- 2 tilt servos, one per motor (this is a bicopter: pitch/roll authority comes partly from
  differential thrust, partly from vectoring each motor's thrust angle via its servo)
- MPU6050 IMU(s)
- A barometric pressure sensor — BMP581 (chosen in Milestone 4, see
  [docs/hardware.md](docs/hardware.md#bmp581-milestone-4)); the estimator must not be coupled to
  a specific part (see [docs/estimation.md](docs/estimation.md))
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
- **Attitude representation:** `flight_core/math/`'s `Quaternion` type (Hamilton product,
  scalar-first, body-to-world; see [docs/math.md](docs/math.md)) is both the internal and
  published representation used by `flight_core/estimation/`'s attitude estimator as of
  Milestone 8 (see [docs/estimation.md](docs/estimation.md)).

## Build instructions

`firmware/` is buildable as of Milestone 2, `flight_core/` as of Milestone 7, and `simulator/`
(as a library, not yet the interactive executable) as of Milestone 9. This section documents the
build story per component; it will be filled in as each remaining piece lands.

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

A standalone static library, plain CMake project (no ESP-IDF dependency), consumable both by
`firmware/` (via ESP-IDF's CMake component system, once a milestone wires that in) and by
`simulator/` (via a normal desktop CMake build, real as of Milestone 9). `math/` (vectors,
quaternions, small matrices — see [docs/math.md](docs/math.md)), `estimation/` (the attitude
estimator — see [docs/estimation.md](docs/estimation.md)), `vehicle/` (`VehicleParams`, the
shared vehicle-constants config — see [docs/dynamics.md](docs/dynamics.md)), and `control/` (the
`Pid` building block, `RateController`, and `AttitudeController` — see
[docs/control.md](docs/control.md)) are real as of Milestones 7-11; `dynamics/safety/` remain
unimplemented.

```sh
cmake -S flight_core -B flight_core/build
cmake --build flight_core/build
```

### simulator/ (desktop physics simulator)

`simulator/physics/` (Milestone 9) is a real, unit-tested rigid-body forward-dynamics model —
see [docs/dynamics.md](docs/dynamics.md). `simulator/sensors/`/`simulator/visualization/` and the
interactive `bicopter_sim` executable (`simulator/main.cpp`) remain unimplemented — a smoke-test
executable was judged unnecessary given the automated test coverage under `tests/`, so building
`simulator/` standalone currently produces just the `bicopter_physics` library, not a runnable
program:

```sh
cmake -S simulator -B simulator/build
cmake --build simulator/build   # builds libbicopter_physics.a; no bicopter_sim executable yet
```

### tests/

Host-side tests for the sensors component's pure conversion/calibration/filtering/stale-detection
logic (MPU6050 and BMP581), the actuators component's pure clamping/pulse-mapping logic (PWM
ESC/servo), `firmware/main/`'s one piece of pure task logic (SensorTask's barometer-read
decimation check), `flight_core/math/`'s vector/matrix/quaternion library,
`flight_core/estimation/`'s complementary-filter attitude estimator (Milestone 8),
`simulator/physics/`'s rigid-body dynamics model (Milestone 9), and `flight_core/control/`'s
`Pid` building block, `RateController` (Milestone 10), and `AttitudeController` (Milestone 11) —
all built independently of ESP-IDF via plain CMake/CTest. The `flight_core` and `bicopter_physics` tests link the real
static libraries (via `add_subdirectory`, see `tests/CMakeLists.txt`); the firmware driver tests
still compile ESP-IDF-free `.c` sources directly by relative path, per
[AGENTS.md](AGENTS.md#driver-testing-convention).

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

## Documentation

- [TODO.md](TODO.md) — engineering roadmap, 18 milestones
- [docs/architecture.md](docs/architecture.md) — pipeline, data flow, HAL design, FreeRTOS tasks
- [docs/math.md](docs/math.md) — quaternion/Euler/rotation-matrix conventions, `flight_core/math/` scope
- [docs/firmware.md](docs/firmware.md) — ESP-IDF toolchain version, build steps, QEMU boot verification
- [docs/hardware.md](docs/hardware.md) — hardware assumptions and how they're made configurable
- [docs/control.md](docs/control.md) — `Pid`, `RateController`, `AttitudeController` design and
  the quaternion attitude-error convention
- [docs/estimation.md](docs/estimation.md) — attitude estimator algorithm and derivation
- [docs/dynamics.md](docs/dynamics.md) — rigid-body dynamics model, vehicle-geometry assumptions,
  and integration scheme (`simulator/physics/`, `flight_core/vehicle/`)
- [AGENTS.md](AGENTS.md) — structural/convention decisions for engineers and agents working on
  later milestones
