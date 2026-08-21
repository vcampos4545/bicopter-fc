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

As of Milestone 7: `firmware/` is a real, buildable ESP-IDF project (see below) whose `main/` runs
six FreeRTOS tasks (SensorTask, EstimatorTask, FlightControlTask, RadioTask, TelemetryTask,
SafetyTask) at documented priorities/periods (`firmware/main/task_config.h`,
[docs/architecture.md](docs/architecture.md#freertos-tasks)), with a queue, a task notification,
an `esp_timer`, a mutex, and the task watchdog wired between them. Task *bodies* are still stub/
no-op logic — no estimation, control-loop math, or actuator-driving logic exists in any task yet;
that is milestones 8-12's job. `SensorTask` is the one exception that reaches real driver code: it
calls the real MPU6050/BMP581 driver interfaces from `firmware/components/sensors/` behind a new
`CONFIG_BICOPTER_SENSORS_ENABLED` Kconfig option (default off, since no board is chosen yet) and
runs gracefully without hardware when it's off. `actuators_init_safe()`
(`firmware/components/actuators/`) is still not called from `main/` — it needs a per-board PWM
pin config that doesn't exist yet (see [docs/hardware.md](docs/hardware.md)'s open ESC/servo pin
items); the natural call site is documented in `firmware/main/main.c` for whichever milestone
finalizes the board config. No radio code exists yet. `flight_core/CMakeLists.txt` builds a real
static library — `flight_core/math/` (vectors, quaternions, small matrices; see
[docs/math.md](docs/math.md), Milestone 7) and, as of Milestone 8, `flight_core/estimation/` (a
Mahony-style nonlinear complementary-filter attitude estimator behind the `AttitudeEstimator`
interface, see [docs/estimation.md](docs/estimation.md)) are real content. Nothing in `firmware/` consumes `flight_core` yet — `EstimatorTask`
(`firmware/main/estimator_task.c`) still only logs/discards each sensor sample; wiring it to
actually call `ComplementaryFilterEstimator` is a noted follow-up (needs `flight_core` wired as
an ESP-IDF component plus a C/C++ boundary adapter converting `imu_reading_t`'s microsecond
timestamps to `ImuSample`'s SI-seconds ones — see docs/estimation.md's "Firmware wiring" section),
not yet done. As of Milestone 9, `flight_core/vehicle/` adds `VehicleParams`/`MotorParams`
(header-only config data, see [docs/dynamics.md](docs/dynamics.md)), and `simulator/physics/` is
real: `bicopter_physics`, a standalone CMake static library (`simulator/physics/CMakeLists.txt`,
same pattern as `flight_core/CMakeLists.txt`) implementing the rigid-body forward-dynamics model
and linking `flight_core` for `Quaternion`/`Vec3`/`VehicleParams`. `simulator/CMakeLists.txt`
configures `physics/` as a real subdirectory (standalone-buildable via
`cmake -S simulator -B simulator/build`); `simulator/sensors/`, `simulator/visualization/`, and
`simulator/main.cpp` (the interactive `bicopter_sim` executable) remain unimplemented — no
simulated `Imu`/`Barometer` HAL implementations exist yet, so nothing yet drives `flight_core`'s
estimator from simulated data. As of Milestone 10, `flight_core/control/` adds `Pid` (a generic
single-axis PID with configurable gains, output saturation, clamping/conditional-integration
anti-windup, and a configurable derivative-on-measurement-vs-error convention — see
[docs/control.md](docs/control.md)) and `RateController` (three independent `Pid` instances, one
per body axis, consuming `AttitudeEstimate::angular_velocity_radps` and a desired rate, producing
a desired body torque). As of Milestone 11, `flight_core/control/` also adds `AttitudeController`
— a stateless proportional quaternion-feedback outer loop (`q_error =
current.inverse() * desired`; see docs/control.md for why that multiplication order, not the
reverse, is the one consistent with `Quaternion::integrate()`'s right-multiply body-rate
convention) converting a desired/current attitude quaternion pair into a desired body rate,
feeding `RateController::update()`'s `desired_radps` argument unchanged. As of Milestone 12,
`flight_core/control/` also adds `ControlAllocator` (`include/control_allocator.h`,
`src/control_allocator.cpp`) — converting a desired total thrust + body torque into motor1/2
throttle and motor1/2 tilt (Milestone 5's `MotorOutput`/`ServoOutput` normalized units), derived
term-by-term (a small-angle linearization around hover, decoupled into a thrust/roll 2x2 solve
followed by a tilt/pitch/yaw 2x2 solve) from Milestone 9's exact forward-dynamics equations, not
invented; see [docs/control_allocation.md](docs/control_allocation.md) for the full derivation,
including the milestone's central finding that this vehicle's geometry has no pitch-torque
authority near hover unless `VehicleParams::center_of_mass_offset_m` has a nonzero vertical
component (a real hardware/geometry finding for Milestone 17, not an allocator limitation).
Getting this derivation to reuse Milestone 9's exact thrust-direction formula (rather than
duplicating it) required moving `motorThrustDirectionBody()` from `simulator/physics/` down into
`flight_core/vehicle/include/motor_geometry.h` — `simulator/physics/`'s `bicopter_physics` target
links `flight_core`, never the reverse, so shared code has to live on the flight_core side of that
edge (the same reasoning that put `VehicleParams` there at Milestone 9); any later milestone
needing to share code between `flight_core/` and something that depends on it should follow the
same pattern rather than duplicating. As of Milestone 13, `simulator/sim_loop/`'s `SimLoop` calls
the full `Pid`/`RateController`/`AttitudeController`/`ControlAllocator` cascade against Milestone
9's simulated dynamics, fed by `simulator/sensors/`'s new `SimulatedImu` (noisy `ImuSample`s
derived from `RigidBodyState` ground truth) rather than perfect state — this is the milestone
where the whole stack first runs closed-loop, demonstrated by `tests/sim_loop_test.cpp`'s genuine
convergence tests (attitude error dropping below a tolerance and staying there, not touching zero
once) and `simulator/main.cpp`'s (`bicopter_sim`) text-trace demo. Two real, structural findings
from this wiring work — accelerometer correction being provably uninformative during hover for a
body-fixed-thrust vehicle, and gyroscopic cross-coupling leaking into the (structurally
unauthoritative, per Milestone 12) pitch axis during multi-axis maneuvers — are fully documented
in [docs/simulation.md](docs/simulation.md), which also confirms Milestones 10-11's placeholder
control gains converge as configured, without retuning. `FlightControlTask`
(`firmware/main/flight_control_task.c`) still only reads/logs `safety_state_t` — wiring this same
stack into firmware for real hardware remains unstarted, a distinct (and larger, hardware-adjacent)
piece of work from closing the loop in simulation. `docs/control.md` covers the
`Pid`/`RateController`/`AttitudeController` design and links to
[docs/control_allocation.md](docs/control_allocation.md) for `ControlAllocator`.
`docs/estimation.md` covers attitude estimation as of Milestone 8 and its Milestone 13 update on
hover observability; barometer-based altitude/vertical-velocity estimation remains deferred to a
later milestone (noted in docs/estimation.md, not silently missing). See [TODO.md](TODO.md) for
the full milestone sequence and current status.

## CMake: guard a shared subdirectory before add_subdirectory-ing it

As of Milestone 9, more than one top-level CMake entry point can pull in `flight_core`:
`tests/CMakeLists.txt` adds it directly (for the math/estimation tests), and
`simulator/physics/CMakeLists.txt` also needs it (for `Quaternion`/`Vec3`/`VehicleParams`) —
including it from both `tests/CMakeLists.txt` (which nests `simulator/physics/` as a
subdirectory) and standalone (`cmake -S simulator -B simulator/build`, which reaches
`flight_core` only through `simulator/physics/CMakeLists.txt`'s own `add_subdirectory`) would
`add_subdirectory` the same physical `flight_core/` directory twice in one configure, which is a
hard CMake error (duplicate target). The fix, in `simulator/physics/CMakeLists.txt`:

```cmake
if(NOT TARGET flight_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../flight_core ${CMAKE_CURRENT_BINARY_DIR}/flight_core-build)
endif()
```

Any later milestone that reuses a library from more than one entry point (e.g. Milestone 12's
`flight_core/control/` linked from both `tests/` and `simulator/`) should follow the same
guarded pattern rather than assuming there's only ever one path into the directory tree.

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
