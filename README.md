# bicopter-fc

A flight-control system for a bicopter: an ESP32 firmware target, a platform-independent
flight-control/estimation library, and a desktop physics simulator that runs the *identical*
flight-control code as the real vehicle.

This is real embedded flight software (ESP-IDF + FreeRTOS, native C/C++ peripheral APIs — not
Arduino), structured to stay readable for an engineer learning embedded flight software.

**Status:** Milestone 16 (safety, failsafes, and battery monitoring) is complete. `firmware/` is a real ESP-IDF
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
the full derivation. As of Milestone 12, `flight_core/control/` also holds `ControlAllocator` —
converting a desired total thrust and body torque (the direct output shape of Milestones 10-11)
into motor1/2 throttle and motor1/2 tilt, Milestone 5's `MotorOutput`/`ServoOutput` normalized
units. This is the mathematical inverse of Milestone 9's forward-dynamics model, derived
term-by-term (a small-angle linearization around hover) from the exact same geometry, not an
invented mixing matrix — see [docs/control_allocation.md](docs/control_allocation.md) for the
full derivation, including this milestone's central finding that the vehicle's geometry as
currently parameterized has no pitch-torque authority near hover unless
`VehicleParams::center_of_mass_offset_m` has a nonzero vertical component. As of Milestone 13,
`simulator/sensors/` and `simulator/sim_loop/` close that loop end to end in simulation:
`SimulatedImu` synthesizes noise/bias-corrupted `ImuSample`s from Milestone 9's ground-truth
`RigidBodyState`, and `SimLoop` wires estimator -> `AttitudeController` -> `RateController` ->
`ControlAllocator` -> Milestone 9 dynamics into one steppable closed loop, run over simulated time
at `SensorTask`/`FlightControlTask`'s real 500 Hz/250 Hz cadence split. `tests/sim_loop_test.cpp`
demonstrates genuine convergence (attitude error dropping below a documented tolerance and staying
there, not just touching zero once) from disturbed initial attitudes, both for the default
(zero-CoM-offset) vehicle's roll/yaw authority and, in an additional case, for pitch under a
nonzero-CoM-offset configuration. Two real, structural findings surfaced during this work — why
the estimator runs with zero accelerometer correction gain during flight, and why the vehicle
needs nonzero drag to bound gyroscopic cross-axis coupling into the (structurally unauthoritative)
pitch axis — are fully documented in [docs/simulation.md](docs/simulation.md), along with
confirmation that Milestones 10-11's placeholder control gains converge as-is, without retuning.
Wiring this same stack into `firmware/`'s `EstimatorTask`/`FlightControlTask` for real hardware
remains a noted follow-up (see docs/estimation.md and docs/simulation.md) — Milestone 13 closes
the loop in the simulator only. As of Milestone 14, `firmware/components/radio/` implements this
project's first real radio link: `include/radio.h` is a protocol-independent `Radio` HAL interface
(so Milestone 15's RC-receiver implementation is a second backend behind it, not a rewrite), and
`esp_now_radio.c/.h` is the first concrete implementation — Wi-Fi/ESP-NOW init and pairing, a
command packet (sequence, timestamp, throttle/roll/pitch/yaw, arm, flight mode) and a minimal
telemetry echo, both validated (magic/version/checksum, non-finite-float rejection) by
`radio_packet.c/.h`'s pure, ESP-IDF-free logic. ESP-NOW's receive callback runs in the Wi-Fi
driver's own task context (not an ISR, but still foreign context this driver must not do real work
in) and does only a bounds-checked, non-blocking queue post; all real parsing, RFC1982-style
sequence-number staleness/reordering rejection, and packet-loss-percentage tracking happen in a
dedicated processing step called once per cycle from Milestone 6's existing `RadioTask`, gated
behind `CONFIG_BICOPTER_RADIO_ENABLED` (default off, no ground-station peer paired yet). Radio-loss
is exposed through `radio_health_t.link_alive` — Milestone 16 still decides what to *do* about a
lost link. No physical ESP32 pair was available to verify real over-the-air delivery/latency/
loss/RSSI, but `idf.py build` was confirmed to succeed with the radio option both off and on (a
real build against ESP-IDF's `esp_wifi`/`esp_now` libraries, not review alone), and every piece of
pure packet logic has real passing host-side tests — see [docs/radio.md](docs/radio.md) for the
full packet format, callback/task-context writeup, and pairing procedure. As of Milestone 15,
`firmware/components/radio/` also has a second concrete `Radio` implementation: CRSF, decoding a
UART-connected RC receiver's `RC_CHANNELS_PACKED` frames (real CRC8/DVB-S2 validation, byte-level
frame sync, 11-bit channel unpacking) into the same `radio_command_t` shape ESP-NOW produces, with
configurable channel-to-function mapping and endpoint/center calibration. CRSF was chosen over
SBUS specifically because classic SBUS carries no CRC/checksum field at all while CRSF does — this
milestone's explicit requirement — see [docs/radio.md](docs/radio.md) for the full protocol-choice
writeup, including the finding that SBUS's usual inverted-UART objection doesn't actually hold on
this project's ESP32 target. Uses ESP-IDF's UART driver in interrupt/ring-buffer mode, not
busy-polling; `radio.h` required no interface changes. `firmware/main/radio_task.c` wires CRSF in
as a build-time alternative to ESP-NOW (mutually exclusive, enforced by a `#error` guard confirmed
to fire). No physical CRSF receiver was available to verify real channel values/timing, but
`idf.py build` was confirmed to succeed with the CRSF option off, on, and to fail loudly with both
radio options on, and the pure frame/channel logic has 140 passing host-side tests — see
[docs/radio.md](docs/radio.md) for the full writeup. As of Milestone 16,
`firmware/components/safety/` implements the full `BOOT`/`DISARMED`/`ARMED`/`STABILIZE`/
`ALTITUDE_HOLD` (Kconfig-gated off by default)/`FAILSAFE`/`ERROR` flight-mode state machine,
arming preconditions, and failsafe detection/response as plain, ESP-IDF-free C — the same
driver-testing-convention split as every other sensor/actuator/radio driver — so
`firmware/main/safety_task.c` calls it directly with no C++/ESP-IDF-component bridge to build;
`flight_core/safety/` stays an empty scaffold (see [docs/safety.md](docs/safety.md) for why). This
replaces Milestone 6's always-disarmed `SafetyTask` stub with real logic, wired into its existing
100Hz loop: `FAILSAFE` is latching (only clears via explicit disarm, never auto-resumes flight),
`ERROR` is reachable only from `BOOT`/`DISARMED` (in-flight faults always route through the single,
well-tested `FAILSAFE` path instead), and arming requires every documented precondition (valid IMU/
estimator, live radio plus an explicit arm command, battery above LOW, no critical errors, and the
design brief's explicit stationary-during-arming check) with no partial-credit path — the system
never auto-arms. Every required failsafe condition except task-watchdog failure (not representable
at the application level, per an existing Milestone 6 finding) is detected with a configurable,
per-condition response defaulting to a safe motor shutdown/disarm, never an attempted autonomous
landing, per the design brief; radio loss fires deterministically off `radio_health_t.link_alive`
alone. `firmware/components/power/` adds ADC-based battery voltage monitoring (ESP32's
line-fitting calibration scheme, its only supported one), a configurable voltage-divider ratio (no
divider chosen yet), an explicitly-approximate voltage-to-percentage curve, configurable LOW/
CRITICAL thresholds, and optional current sensing, gated behind
`CONFIG_BICOPTER_BATTERY_ENABLED` (default off) and failing closed while disabled. Real live-data
wiring this milestone: IMU/barometer validity and raw gyro rate (peeked from `SensorTask`'s sample
queue), radio arm command and link health (`RadioTask` now publishes via a new `radio_state.h`,
its first live cross-task consumer), and battery data when enabled. Honestly not wired: estimator
validity and attitude, since no estimator runs in firmware yet (the same open item Milestones
8/10-13 already track) — `estimator_valid` is hardcoded false, never faked true, so arming is
structurally blocked in today's firmware regardless of hardware state; see
[docs/safety.md](docs/safety.md)'s full breakdown. `idf.py build` succeeds with the battery/
current-sense/altitude-hold options each off (default) and on; 185 new passing host-side checks
cover the full transition table, every arming precondition, every failsafe condition, and the
battery conversion math (`tests/flight_mode_test.c`, `tests/arming_test.c`,
`tests/failsafe_test.c`, `tests/actuator_command_check_test.c`, `tests/battery_convert_test.c`).
No physical hardware was available to verify actual I2C/PWM/radio/ADC transactions or
real-hardware task timing. As of Milestone 18 - the last purely-software milestone before
Milestone 17's hardware integration, which is captain-directed and requires physical hardware in
hand (see [TODO.md](TODO.md)) - the firmware adds `SIMULATION`/`HARDWARE_TEST`/`FLIGHT` build-time
operating modes (a Kconfig choice) and a `BENCH_TEST` build configuration that `#if`-excludes
every LEDC call in `firmware/components/actuators/src/pwm_esc_output.c` so no motor-spinning PWM
signal can be produced in that build (verified by `nm`-inspecting the compiled object, not just
documented), plus a new `firmware/components/bench_test/` UART console (`BenchTestTask`) with four
independently-usable bench-bringup commands - continuous sensor/radio streaming, a direct
single-servo angle command, and a single-motor `esc_test` reachable only in a `HARDWARE_TEST`
build and gated behind an explicit, exact `CONFIRM` token distinct from the normal arm/throttle
path. This milestone also wires `actuators_init_safe()` into `main()` for the first time, behind a
new `CONFIG_BICOPTER_ACTUATORS_ENABLED` gate. See [docs/bench_test.md](docs/bench_test.md) for the
full writeup, including the recommended real-hardware bring-up sequence Milestone 17 is meant to
run against — see [TODO.md](TODO.md) for the full roadmap and what's implemented so far.

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
- A radio link — ESP-NOW (Milestone 14) and CRSF RC receiver (Milestone 15), selectable at build
  time
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
estimator — see [docs/estimation.md](docs/estimation.md)), `vehicle/` (`VehicleParams` and the
shared `motorThrustDirectionBody()` geometry helper — see [docs/dynamics.md](docs/dynamics.md)),
and `control/` (the `Pid` building block, `RateController`, `AttitudeController`, and — as of
Milestone 12 — `ControlAllocator` — see [docs/control.md](docs/control.md) and
[docs/control_allocation.md](docs/control_allocation.md)) are real as of Milestones 7-12;
`dynamics/safety/` remain unimplemented.

```sh
cmake -S flight_core -B flight_core/build
cmake --build flight_core/build
```

### simulator/ (desktop physics simulator)

`simulator/physics/` (Milestone 9) is a real, unit-tested rigid-body forward-dynamics model — see
[docs/dynamics.md](docs/dynamics.md). As of Milestone 13, `simulator/sensors/` (`SimulatedImu`)
and `simulator/sim_loop/` (`SimLoop`, the estimator -> attitude/rate control -> allocation ->
dynamics closed loop) are real too, and `simulator/main.cpp` builds as the interactive
`bicopter_sim` executable — a minimal text-trace demo of closed-loop stabilization; see
[docs/simulation.md](docs/simulation.md). A later follow-up task (not one of the 18 numbered
milestones) added `simulator/visualization/`: a real graphical renderer of the same closed loop,
built on the captain's [VGL](https://github.com/vcampos4545/VGL) library and pulled in via CMake
`FetchContent`, additive alongside (not replacing) the text-trace demo — see
[docs/visualization.md](docs/visualization.md).

```sh
cmake -S simulator -B simulator/build
cmake --build simulator/build
./simulator/build/bicopter_sim                      # text-trace demo of closed-loop stabilization
./simulator/build/visualization/bicopter_sim_viz     # graphical demo (requires: brew install glfw glew glm)
```

### tests/

Host-side tests for the sensors component's pure conversion/calibration/filtering/stale-detection
logic (MPU6050 and BMP581), the actuators component's pure clamping/pulse-mapping logic (PWM
ESC/servo), `firmware/main/`'s one piece of pure task logic (SensorTask's barometer-read
decimation check), `flight_core/math/`'s vector/matrix/quaternion library,
`flight_core/estimation/`'s complementary-filter attitude estimator (Milestone 8),
`simulator/physics/`'s rigid-body dynamics model (Milestone 9), `flight_core/control/`'s
`Pid` building block, `RateController` (Milestone 10), `AttitudeController` (Milestone 11), and
`ControlAllocator` (Milestone 12), — as of Milestone 13 — `simulator/sim_loop/`'s full
closed-loop convergence tests (`sim_loop_test`, see [docs/simulation.md](docs/simulation.md)), and
— as of Milestone 14 — the radio component's pure packet-format/sequence-staleness/packet-loss
logic (`radio_packet_test`, see [docs/radio.md](docs/radio.md)), and — as of Milestone 15 — the
radio component's pure CRSF CRC8/frame-sync/channel-decode/calibration logic
(`crsf_frame_test`, see [docs/radio.md](docs/radio.md)), and — as of Milestone 16 — the safety
component's pure flight-mode-transition/arming-precondition/failsafe-evaluation/actuator-command-
validation logic (`flight_mode_test`, `arming_test`, `failsafe_test`,
`actuator_command_check_test`) and the power component's pure battery ADC-conversion logic
(`battery_convert_test`) — see [docs/safety.md](docs/safety.md) — and, as of Milestone 18, the new
bench_test component's pure UART command-line-parsing logic (`bench_test_command_test`) and the
BENCH_TEST motor-disable gate's compile-time macro translation, built twice under both
configurations (`pwm_esc_bench_test_gate_test`, `pwm_esc_bench_test_gate_disabled_test`) — see
[docs/bench_test.md](docs/bench_test.md) — and, from the visualization follow-up task, the
`simulator/visualization/` NED<->render-space coordinate conversion (`coordinate_convert_test`,
glm/VGL-free by design — see [docs/visualization.md](docs/visualization.md)) — all built
independently of ESP-IDF via plain CMake/CTest. The `flight_core` and `bicopter_physics`
tests link the real static libraries (via `add_subdirectory`, see `tests/CMakeLists.txt`);
`control_allocator_test` additionally links `bicopter_physics` (not just `flight_core`) so its
round-trip checks can call Milestone 9's real forward-dynamics function directly; `sim_loop_test`
links `bicopter_sim_loop`, which itself pulls in `bicopter_physics`, `bicopter_sim_sensors`, and
`flight_core`; the firmware driver tests still compile ESP-IDF-free `.c` sources directly by
relative path, per [AGENTS.md](AGENTS.md#driver-testing-convention).

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
- [docs/control_allocation.md](docs/control_allocation.md) — `ControlAllocator` derivation from
  Milestone 9's forward dynamics, controllability-near-hover findings, and saturation policy
- [docs/estimation.md](docs/estimation.md) — attitude estimator algorithm and derivation
- [docs/dynamics.md](docs/dynamics.md) — rigid-body dynamics model, vehicle-geometry assumptions,
  and integration scheme (`simulator/physics/`, `flight_core/vehicle/`)
- [docs/simulation.md](docs/simulation.md) — closed-loop wiring (`simulator/sim_loop/`), the
  simulated IMU (`simulator/sensors/`), convergence criteria, and this milestone's findings on
  accelerometer observability during hover and gyroscopic pitch coupling
- [docs/visualization.md](docs/visualization.md) — the VGL-based graphical visualizer
  (`simulator/visualization/`, `bicopter_sim_viz`), what it renders, the NED<->render-space
  coordinate conversion, and its build/verification story
- [docs/radio.md](docs/radio.md) — `Radio` HAL interface, ESP-NOW packet format and pairing
  procedure, CRSF frame format/protocol-choice rationale/channel-mapping and calibration
  procedure, both backends' callback-or-driver/task-context splits, and packet-loss/staleness
  tracking
- [docs/safety.md](docs/safety.md) — flight-mode state machine and full transition table, arming
  preconditions, every failsafe condition and its default response, and the battery-monitoring
  model/thresholds
- [docs/bench_test.md](docs/bench_test.md) — the three build-time operating modes, the `BENCH_TEST`
  motor-disable compile-time guarantee and how it's enforced, the bench-test UART console's four
  commands and their safety gating, and the recommended real-hardware bring-up sequence
- [AGENTS.md](AGENTS.md) — structural/convention decisions for engineers and agents working on
  later milestones
