# Roadmap

18 milestones. Each is dispatched as its own task once the previous ones it depends on have
landed. "Definition of done" is the bar for closing that milestone's task, not a full-system
acceptance test — later milestones are expected to revisit earlier code as real constraints
appear (e.g. the estimator milestone may add fields to the `Imu` interface).

- [x] **1. Repository, architecture, and documentation**
  Deliverable: the `firmware/ flight_core/ simulator/ tests/ docs/` structure, README.md,
  this file, and `docs/architecture.md` / `docs/hardware.md` / `docs/control.md` /
  `docs/estimation.md`.
  Done when: structure and docs exist, commit clean, no placeholder flight-control code.

- [x] **2. ESP-IDF boots**
  Deliverable: a minimal `firmware/` ESP-IDF project that builds and flashes, bringing up
  logging and confirming the target board boots to a known-good idle state.
  Done when: `idf.py build` succeeds and the board boots and logs over serial on real hardware
  (or documented reason real hardware wasn't available).
  Done via: ESP-IDF v5.5.5, target `esp32`, boot verified in QEMU (no physical board available
  in this environment) — see [docs/firmware.md](docs/firmware.md) for the toolchain version,
  build/verify steps, and captured boot log.

- [x] **3. MPU6050 driver + tests**
  Deliverable: an ESP-IDF component driving the MPU6050 over I2C (init, calibration read,
  accel/gyro sample retrieval) behind the `Imu` hardware-abstraction interface, with unit tests.
  Done when: the driver reads real accel/gyro samples on hardware and has test coverage for its
  parsing/scaling logic independent of the bus.
  Done via: `firmware/components/sensors/` (`mpu6050.c`/`.h` for I2C/ISR, `mpu6050_convert.c`/
  `.h` for the pure register-to-SI/calibration/stale-detection logic). Data-ready-interrupt-driven
  reads (minimal ISR, hand-off via FreeRTOS queue) with a polled fallback — see
  [docs/hardware.md](docs/hardware.md#mpu6050-milestone-3) for the choice and reasoning.
  `idf.py build` succeeds with the component included; conversion/calibration/staleness logic has
  33 passing host-side tests (`tests/mpu6050_convert_test.c`). No physical MPU6050 was available
  in this environment — real I2C transactions and sensor readings are unverified until hardware
  lands (see docs/hardware.md for the full verified-vs-deferred breakdown).

- [x] **4. Barometer driver**
  Deliverable: an ESP-IDF component driving the barometric pressure sensor (BMP390 or BMP581 —
  whichever is fitted) behind the `Barometer` interface, without coupling any downstream code
  to the specific part.
  Done when: the driver reads pressure/temperature on hardware and altitude derivation is
  covered by a unit test independent of the bus.
  Done via: `firmware/components/sensors/` (`bmp581.c`/`.h` for I2C, `bmp581_convert.c`/`.h` for
  the pure register-to-SI/filtering/altitude/stale-detection logic). BMP581 chosen over BMP390 for
  its simpler register interface (already-compensated output, no OTP trim-coefficient polynomial)
  — see [docs/hardware.md](docs/hardware.md#bmp581-milestone-4) for the choice and reasoning.
  Polled reads (continuous power mode, no data-ready interrupt — baro dynamics are slow enough
  that this wasn't worth the MPU6050 driver's ISR complexity). `idf.py build` succeeds with the
  component included; pressure/temperature conversion, EMA pressure filtering, barometric-formula
  altitude derivation, and staleness logic have 14 passing host-side test groups
  (`tests/bmp581_convert_test.c`). No physical BMP581 was available in this environment — real I2C
  transactions and sensor readings are unverified, and the register map itself carries lower
  confidence than Milestone 3's MPU6050 driver (see docs/hardware.md for the full
  verified-vs-deferred breakdown).

- [x] **5. ESC + servo hardware abstraction**
  Deliverable: `MotorOutput` and `ServoOutput` implementations driving the BLHeli ESCs and tilt
  servos (PWM/DShot as appropriate), with configurable protocol/range/calibration.
  Done when: commanded values on hardware produce correct, safely-bounded ESC/servo output,
  verified on a bench (props off).
  Done via: `firmware/components/actuators/` - `pwm_esc_output.c/.h` (conventional RC PWM over
  LEDC, implementing the generic `motor_output_t` vtable interface in `motor_output.h` so a future
  DShot implementation is a drop-in addition) and `servo_output.c/.h` (LEDC PWM tilt-servo
  output), each paired with a pure `*_convert.c/.h` module (throttle/angle clamping,
  pulse-width mapping) plus the shared `pwm_util.c/.h` (pulse-width-to-LEDC-duty-count math).
  `actuators_init_safe()` (`actuators_init.h`) is the explicit safe-init entry point that leaves
  every motor idle and every servo neutral before returning (not yet wired into `main/` - that's
  milestone 6). See [docs/hardware.md](docs/hardware.md#pwm-esc--tilt-servo-milestone-5) for the
  LEDC-vs-MCPWM choice, ESC protocol-determination and calibration guidance, and the full
  verified-vs-deferred breakdown. `idf.py build` succeeds with the component included;
  clamping/pulse-mapping/duty-count math has 61 passing host-side checks across
  `tests/pwm_esc_convert_test.c`, `tests/servo_convert_test.c`, and `tests/pwm_util_test.c`. No
  physical ESC, servo, or motor was available in this environment, and nothing was armed - all
  hardware verification is deferred (see docs/hardware.md).

- [x] **6. FreeRTOS task architecture**
  Deliverable: the task skeleton described in `docs/architecture.md` (SensorTask, EstimatorTask,
  FlightControlTask, RadioTask, TelemetryTask, SafetyTask) wired up with their target
  rates/priorities and the queues/synchronization between them, running with stub payloads.
  Done when: all tasks run concurrently at their target rates without missed deadlines or
  priority inversions, measured on hardware.
  Done via: `firmware/main/` - six tasks at concrete priorities/periods (`task_config.h`), a
  length-1 overwrite queue (`SensorTask` -> `EstimatorTask`), a task notification (`SafetyTask`
  -> `FlightControlTask`), an `esp_timer` driving `TelemetryTask`'s wake, and a mutex around the
  one genuinely cross-task shared state (`safety_state_t`) - see
  [docs/architecture.md](docs/architecture.md#freertos-tasks) for the full table and reasoning.
  `SensorTask` calls the real milestone 3/4 driver interfaces behind a new
  `CONFIG_BICOPTER_SENSORS_ENABLED` Kconfig option (default off, no board chosen yet) and
  gracefully runs without hardware when it's off. The task watchdog is enabled
  (`CONFIG_ESP_TASK_WDT_PANIC=y`, 1s timeout) and all six tasks feed it. `idf.py build` succeeds;
  QEMU boot confirms all six tasks running concurrently at their target rates with no crash, no
  FreeRTOS assertion, and no watchdog timeout over a ~12s run - not a substitute for real-hardware
  timing verification, which remains milestone 17's job. One new host test
  (`tests/task_util_test.c`) covers the one piece of pure logic this milestone added; the rest is
  FreeRTOS/ESP-IDF scaffolding not meaningfully testable off-target. No physical board was
  available in this environment (same constraint as every milestone so far - see
  docs/hardware.md).

- [x] **7. Math library**
  Deliverable: `flight_core/math/` — vectors, quaternions/rotations, and any numerical utilities
  the estimator and controllers need, platform-independent and unit-tested.
  Done when: the library has test coverage for rotation composition, normalization, and the
  operations later milestones depend on.
  Done via: `flight_core/math/` (`vec3.h/.cpp`, `mat3.h/.cpp`, `quaternion.h/.cpp`) — a `Vec3`
  (add/subtract/scale/dot/cross/normalize), a fixed 3x3 `Mat3` (matrix-vector/matrix-matrix
  multiply, transpose, `FromDiagonal` for an inertia tensor), and a `Quaternion` (identity,
  Hamilton-product multiplication, conjugate/inverse, normalization, exponential-map body-rate
  integration, ZYX-Euler and rotation-matrix conversion including a documented gimbal-lock
  convention) — all pure C++17, zero ESP-IDF/OS dependency, `float`-precision throughout. See
  [docs/math.md](docs/math.md) for the full quaternion/Euler-sequence/axis-convention writeup
  reconciling this with README.md's body-frame convention. `flight_core/CMakeLists.txt` now
  builds a real static library (`add_library(flight_core STATIC ...)`, `cxx_std_17`); `tests/`
  links it in via `add_subdirectory` and adds `vec3_test.cpp`/`mat3_test.cpp`/
  `quaternion_test.cpp` — 92 passing checks across 9 CTest suites total, including property tests
  ("integrate a constant angular velocity for a known time" and "Euler round-trip, with the
  pitch=+-90 gimbal-lock case tested and documented separately"). `idf.py build` still succeeds
  (firmware/ untouched this milestone); no estimator, controller, or dynamics-simulation logic was
  added.

- [x] **8. State estimator**
  Deliverable: `flight_core/estimation/` — an attitude (and altitude) estimator fusing IMU and
  barometer data into the vehicle state, per the approach chosen and documented in
  `docs/estimation.md`.
  Done when: the estimator converges to a correct attitude estimate against recorded or
  simulated sensor data, with unit tests.
  Done via: `flight_core/estimation/` — `AttitudeEstimator` (`include/attitude_estimator.h`), an
  abstract interface (`reset`/`update`/`estimate`) so a future EKF can be a drop-in replacement,
  and `ComplementaryFilterEstimator` (`include/complementary_filter.h`, `src/complementary_filter.cpp`),
  a Mahony-style nonlinear complementary filter: gyro integration via Milestone 7's
  `Quaternion::integrate()`, corrected by an accelerometer-derived gravity-direction error fed back
  as an angular-rate correction, gated to zero weight when `|accel|` deviates too far from 1g (the
  classic complementary-filter high-acceleration failure mode). Real timestamps (not a fixed dt),
  configurable static bias offsets plus optional online bias adaptation. Attitude only — altitude/
  vertical-velocity estimation from the barometer is explicitly deferred, see
  [docs/estimation.md](docs/estimation.md). `flight_core/CMakeLists.txt` now also builds
  `estimation/src/complementary_filter.cpp`; `tests/complementary_filter_test.cpp` (linked against
  the real `flight_core` library, same as Milestone 7's math tests) covers stationary convergence
  from a bad initial attitude, known constant-rotation integration, noisy-input robustness, and the
  high-acceleration accel-rejection behavior — 4513 passing checks. `docs/estimation.md` has the
  full algorithm derivation, the EKF-drop-in interface contract, and why a complementary filter was
  chosen over an EKF for this milestone. `idf.py build` still succeeds (`firmware/` untouched);
  wiring `EstimatorTask` to actually call this estimator is left as an explicitly-noted follow-up
  (needs `flight_core` wired as an ESP-IDF component plus a C/C++ boundary adapter — see
  docs/estimation.md's "Firmware wiring" section), not attempted this milestone.

- [x] **9. Bicopter dynamics simulator**
  Deliverable: `simulator/physics/` — a rigid-body dynamics model of the bicopter (mass,
  inertia, motor thrust/torque, servo tilt geometry) driving simulated `Imu`/`Barometer`
  implementations that `flight_core` reads through the same HAL interfaces used on hardware.
  Done when: the simulator runs an open-loop physics integration that responds plausibly to
  fixed motor/servo commands (e.g. correct free-fall and torque response), independent of any
  controller.
  Done via: `simulator/physics/` (`include/bicopter_dynamics.h`, `src/bicopter_dynamics.cpp`) —
  `computeStateDerivative()` implements `m*v_dot=F` and `I*omega_dot + omega x (I*omega) = tau`
  (gravity + per-motor thrust/reaction-torque + simple linear drag; diagonal-inertia solve for
  `omega_dot`, see [docs/dynamics.md](docs/dynamics.md) for why), and `stepRigidBodyState()`
  integrates forward with semi-implicit Euler for translation/angular-velocity and
  `Quaternion::integrate()` (Milestone 7) for orientation. Vehicle-specific constants (mass,
  diagonal inertia, motor arm offsets/spin direction/tilt limits/thrust+torque coefficients,
  drag coefficients) live in `flight_core/vehicle/include/vehicle_params.h`
  (`VehicleParams`/`MotorParams`) per docs/hardware.md's Milestone-1 design note that this
  config is meant to be shared with Milestone 12's control allocation, not duplicated. Every
  geometry/coefficient assumption (tilt-vectoring axis and sign convention, quadratic
  thrust-vs-throttle curve, thrust-proportional reaction torque, independently-configurable
  motor spin direction, diagonal-inertia rationale, linear drag model) is derived and documented
  in [docs/dynamics.md](docs/dynamics.md), not invented ad hoc. `tests/bicopter_dynamics_test.cpp`
  (linked against a new `bicopter_physics` static library, `simulator/physics/CMakeLists.txt`,
  itself linking `flight_core`) covers free-fall kinematics, symmetric-thrust hover equilibrium
  (zero net acceleration and torque), asymmetric-thrust- and tilt-induced torque checked against
  hand-derived closed forms, reaction-torque-driven yaw, and a torque-free angular-momentum
  conservation check — 2034 passing checks. Simulated sensor noise/bias models and
  `simulator/sensors/` remain explicitly deferred (see docs/dynamics.md's Scope section);
  `simulator/main.cpp`/the `bicopter_sim` executable also remain unbuilt (a smoke-test executable
  was judged unnecessary given the automated test coverage) — `simulator/CMakeLists.txt` now
  configures `physics/` as a real subdirectory, standalone-buildable
  (`cmake -S simulator -B simulator/build`) as well as via `tests/`. `idf.py build` still
  succeeds (`firmware/` untouched this milestone).

- [ ] **10. PID / rate controller**
  Deliverable: `flight_core/control/` — a rate (angular-velocity) controller (PID or equivalent)
  taking estimator output and rate setpoints to torque/force demands.
  Done when: unit-tested against synthetic rate-error inputs and produces stable, bounded
  output.

- [ ] **11. Attitude controller**
  Deliverable: an attitude (angle) control loop generating rate setpoints for the rate
  controller from attitude setpoints and estimator output.
  Done when: unit-tested for stability and bounded output across a representative range of
  attitude errors.

- [ ] **12. Control allocation**
  Deliverable: `flight_core/control/` allocation logic mapping desired body torques/thrust to
  the two motors' throttle and two servos' tilt angles, derived from the actual bicopter
  force/torque geometry documented in `docs/control.md` at that time (not invented mixing
  coefficients).
  Done when: allocation math is derived from the vehicle geometry, unit-tested against known
  torque/thrust demands, and respects actuator limits.

- [ ] **13. Simulator closed-loop stabilization**
  Deliverable: the full flight_core stack (estimator + attitude/rate control + allocation)
  running unmodified inside `simulator/`, closing the loop against the physics model from
  milestone 9.
  Done when: the simulated vehicle self-stabilizes to a commanded attitude from a perturbed
  initial state, demonstrated in the simulator (with visualization if available by then).

- [ ] **14. ESP-NOW**
  Deliverable: `firmware/components/radio/` ESP-NOW link implementing the `Radio` interface for
  telemetry down and setpoint/command up.
  Done when: two ESP32 boards exchange setpoint and telemetry packets over ESP-NOW with measured
  latency/loss characteristics documented.

- [ ] **15. RC receiver abstraction**
  Deliverable: a second `Radio`-interface (or parallel input) implementation for a conventional
  RC receiver (PPM/SBUS/etc., protocol TBD), selectable alongside or instead of ESP-NOW.
  Done when: stick/channel input from a real RC receiver is decoded into the same setpoint
  representation ESP-NOW produces, on hardware.

- [ ] **16. Safety / failsafes**
  Deliverable: `flight_core/safety/` + `firmware/components/safety/` — arming logic, signal-loss
  and sensor-fault failsafes, battery-voltage cutoffs, and the SafetyTask behavior.
  Done when: each failsafe condition (radio loss, sensor fault, low battery, disarm) is
  exercised in the simulator and/or on the bench and produces the correct safe response.

- [ ] **17. Hardware integration**
  Deliverable: all of the above assembled and flown/tested on the finalized physical vehicle,
  including any wiring/mounting/calibration documentation needed to reproduce the build.
  Done when: the vehicle achieves stable controlled flight (or the tethered/bench equivalent
  agreed for first hardware tests).

- [ ] **18. Bench-test tooling**
  Deliverable: `tests/` and/or standalone tooling for bench validation — motor/servo output
  checks, sensor calibration routines, log capture/analysis — usable without full flight.
  Done when: a documented bench procedure exists for validating each subsystem in isolation
  before flight.
