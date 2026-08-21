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

- [x] **10. PID / rate controller**
  Deliverable: `flight_core/control/` — a rate (angular-velocity) controller (PID or equivalent)
  taking estimator output and rate setpoints to torque/force demands.
  Done when: unit-tested against synthetic rate-error inputs and produces stable, bounded
  output.
  Done via: `flight_core/control/` — `Pid` (`include/pid.h`, `src/pid.cpp`), a generic single-axis
  PID with configurable gains, configurable output saturation, clamping (conditional-integration)
  anti-windup, a configurable derivative-on-measurement (default, avoids setpoint-step "derivative
  kick") vs. derivative-on-error convention, and an explicit per-call `dt` (never a fixed internal
  rate — the same `Pid` type will back Milestone 11's attitude loop at a different rate). Built on
  top of it, `RateController` (`include/rate_controller.h`, `src/rate_controller.cpp`) wraps three
  independent `Pid` instances (roll/pitch/yaw rate) with zero cross-axis coupling inside the
  controller itself — `update(desired_radps, current_radps, dt) -> Vec3 torque_nm`, consuming
  Milestone 8's `AttitudeEstimate::angular_velocity_radps` shape and producing a body torque, not
  yet a motor/servo command (that's Milestone 12's control allocation). See
  [docs/control.md](docs/control.md) for the full anti-windup/derivative-convention reasoning and
  why the default gains are explicitly labeled placeholders (real tuning deferred to Milestone 13's
  simulator closed loop or real hardware, Milestone 17). `flight_core/CMakeLists.txt` now also
  builds `control/src/pid.cpp` and `control/src/rate_controller.cpp`.
  `tests/pid_test.cpp` (39 checks) covers proportional-only response, integral accumulation,
  anti-windup under saturation (including immediate recovery once the error reverses),
  derivative-on-measurement vs. derivative-on-error (including the kick the latter produces and
  the former doesn't), output clamping, and non-positive-`dt` handling.
  `tests/rate_controller_test.cpp` (27 checks) covers axis independence (an isolated roll/pitch/
  yaw rate error produces ~zero torque on the other two axes), agreement with an independently-run
  `Pid` per axis across multiple steps, and `reset()`. No attitude-loop or control-allocation code
  was added. `idf.py build` still succeeds (`firmware/` untouched this milestone).

- [x] **11. Attitude controller**
  Deliverable: an attitude (angle) control loop generating rate setpoints for the rate
  controller from attitude setpoints and estimator output.
  Done when: unit-tested for stability and bounded output across a representative range of
  attitude errors.
  Done via: `flight_core/control/` — `AttitudeController` (`include/attitude_controller.h`,
  `src/attitude_controller.cpp`), a stateless proportional quaternion-feedback outer loop:
  `q_error = current.inverse() * desired` (derived from, and consistent with,
  `Quaternion::integrate()`'s right-multiply body-rate convention — see docs/control.md for the
  full derivation and why the opposite multiplication order silently flips control direction),
  a double-cover shortest-path fix (negate `q_error` when its `w < 0`), and a small-angle
  proportional law `commanded = 2 * kp .* q_error.{x,y,z}` per axis, feeding
  `RateController::update()`'s `desired_radps` argument unchanged. Configurable per-axis gain
  (`AttitudeControllerConfig::kp`) and an always-on per-axis rate-limit clamp
  (`rate_limit_radps`, non-positive = disabled per axis) bound the outer loop's rate demand. An
  optional `feedforward_radps` argument (default zero) is documented but not yet exercised by any
  real caller. `flight_core/CMakeLists.txt` now also builds `control/src/attitude_controller.cpp`.
  `tests/attitude_controller_test.cpp` (37 checks) covers zero error at both identity and a
  non-trivial matched attitude, each axis's error in isolation (correct sign and magnitude against
  an independently-computed closed form), a combined multi-axis error with distinct per-axis
  gains, feedforward pass-through at zero error, the shortest-path fix for a >180 degree error, and
  rate-limit saturation (exact clamp, other axes unaffected). No control-allocation code was added
  and `RateController`'s own interface was left unmodified. See [docs/control.md](docs/control.md)
  for the full quaternion-error derivation, the small-angle assumption/limitation, and the
  explicitly-undertaken (not full recovery) scope of the 180-degree handling. `idf.py build` still
  succeeds (`firmware/` untouched this milestone).

- [x] **12. Control allocation**
  Deliverable: `flight_core/control/` allocation logic mapping desired body torques/thrust to
  the two motors' throttle and two servos' tilt angles, derived from the actual bicopter
  force/torque geometry documented in `docs/control.md` at that time (not invented mixing
  coefficients).
  Done when: allocation math is derived from the vehicle geometry, unit-tested against known
  torque/thrust demands, and respects actuator limits.
  Done via: `flight_core/control/` — `ControlAllocator` (`include/control_allocator.h`,
  `src/control_allocator.cpp`), converting a desired total thrust + body torque (the direct output
  shape of Milestones 10-11) into motor1/2 throttle and motor1/2 tilt, in Milestone 5's
  `MotorOutput`/`ServoOutput` normalized units. The allocation math is a small-angle linearization
  of Milestone 9's exact forward-dynamics equations around hover, decoupled into two independent
  2x2 linear solves (total-thrust+roll -> per-motor thrust, then pitch+yaw -> per-motor tilt given
  those thrusts) — every coefficient traces to a `VehicleParams` field, none invented. See
  [docs/control_allocation.md](docs/control_allocation.md) for the full term-by-term derivation.
  `motorThrustDirectionBody()` moved from `simulator/physics/` to
  `flight_core/vehicle/include/motor_geometry.h` this milestone so the allocator could call the
  exact same function Milestone 9 already tests, rather than duplicating it (`simulator/physics/`
  still re-exposes it unchanged for existing callers). The milestone's central finding: with this
  vehicle's geometry (both motors on body Y, tilt axis also body Y) and
  `VehicleParams::center_of_mass_offset_m` at its default `Vec3::Zero()`, this configuration has NO
  pitch-torque authority near hover — a provable structural fact, not an allocator limitation; real
  pitch authority requires a nonzero vertical CoM/motor-plane offset, which the allocator uses
  correctly when configured and honestly reports as unachievable (via `AllocatedCommand::saturated`)
  when not, rather than silently doing nothing. Saturation policy: motor throttle bounds are hard
  limits; total thrust is preserved over roll accuracy when they conflict (losing lift is worse
  than losing some roll authority); tilt/pitch/yaw saturate independently per-servo. Degenerate
  geometry (near-zero determinants) falls back to documented, non-crashing behavior rather than
  dividing by zero. `tests/control_allocator_test.cpp` (144 checks) covers hover, isolated
  roll/pitch/yaw, the pitch-achievable-vs-unachievable CoM-offset comparison, saturation (excess/
  negative thrust, extreme roll/yaw, adversarial-input finiteness, degenerate geometry), and —the
  single most important test in this milestone — a combined multi-axis command whose allocator
  output is fed back through Milestone 9's real `computeStateDerivative()` to confirm the derived
  inverse actually reproduces the requested thrust/torque. `idf.py build` still succeeds
  (`firmware/` untouched this milestone).

- [x] **13. Simulator closed-loop stabilization**
  Deliverable: the full flight_core stack (estimator + attitude/rate control + allocation)
  running unmodified inside `simulator/`, closing the loop against the physics model from
  milestone 9.
  Done when: the simulated vehicle self-stabilizes to a commanded attitude from a perturbed
  initial state, demonstrated in the simulator (with visualization if available by then).
  Done via: `simulator/sensors/` — `SimulatedImu` (`include/simulated_imu.h`,
  `src/simulated_imu.cpp`), producing noise/bias-corrupted `ImuSample`s from Milestone 9's
  `RigidBodyState` ground truth at a configurable, fixed sample rate (default 500 Hz, matching
  `SensorTask`). `simulator/sim_loop/` — `SimLoop` (`include/sim_loop.h`, `src/sim_loop.cpp`)
  wires estimator -> `AttitudeController` -> `RateController` -> `ControlAllocator` -> Milestone
  9 dynamics into one steppable loop, at `SensorTask`/`FlightControlTask`'s real 500 Hz/250 Hz
  cadence split (zero-order-hold actuator commands between control cycles). `simulator/main.cpp`
  (`bicopter_sim`) is a minimal text-trace demo (no graphical visualization — out of scope for
  this numbered milestone). Two real, structural findings surfaced and are fully documented in
  [docs/simulation.md](docs/simulation.md): (1) this vehicle's body-fixed thrust makes
  accelerometer-based gravity-direction correction provably uninformative during hover (an
  extension of Milestone 8's "yaw unobservable from gravity" finding to roll/pitch as well), so
  the closed loop runs the estimator with `kp=0` (pure gyro integration) during flight, seeded
  from the true initial attitude at arm time (modeling real pre-arm accelerometer calibration);
  (2) a combined roll+yaw maneuver excites gyroscopic cross-coupling that leaks into pitch, an
  axis the default (zero CoM offset) vehicle has no torque authority over (Milestone 12's
  finding) — nonzero drag (anticipated by docs/dynamics.md for exactly this milestone) bounds
  that leakage instead of letting it grow unboundedly. Milestones 10-11's placeholder
  `Pid`/`RateController`/`AttitudeController` gains were validated, not retuned — they converge
  all tested cases once the above were fixed (see docs/simulation.md's "Gain tuning" section for
  why this is reported honestly rather than re-tuned for its own sake).
  `tests/sim_loop_test.cpp` (12 checks, 3 real convergence cases, not single-step sanity checks):
  a 20° roll disturbance and a combined 15° roll + 20° yaw disturbance, both recovering to level
  under the default (zero CoM offset) vehicle — respecting Milestone 12's no-pitch-authority
  finding rather than contradicting it — plus an additional 15° pitch disturbance recovering to
  level under a nonzero-CoM-offset vehicle, demonstrating real pitch authority when configured.
  Each case asserts the TRUE (ground-truth, not estimated) attitude error drops below a
  documented tolerance and *stays* there for the rest of a multi-second run, not just touches
  zero once. `idf.py build` still succeeds (`firmware/` untouched — this milestone is
  simulator-only).

- [x] **14. ESP-NOW**
  Deliverable: `firmware/components/radio/` ESP-NOW link implementing the `Radio` interface for
  telemetry down and setpoint/command up.
  Done when: two ESP32 boards exchange setpoint and telemetry packets over ESP-NOW with measured
  latency/loss characteristics documented.
  Done via: `firmware/components/radio/` — `include/radio.h`, a protocol-independent `Radio` HAL
  interface (`has_command`/`get_command`/`get_health`/`deinit`, the same ops-vtable-plus-ctx shape
  as `motor_output_t`) so Milestone 15's RC-receiver implementation is a second backend behind the
  same interface, not a rewrite. `esp_now_radio.c/.h` is the first concrete implementation: Wi-Fi
  station + ESP-NOW init/pairing, a 33-byte command packet (sequence, timestamp,
  throttle/roll/pitch/yaw, arm, flight mode) and a 21-byte telemetry echo packet, both with a
  magic/version/checksum guard against a foreign or corrupted packet. `radio_packet.c/.h` holds
  the pure wire-format serialize/parse/validation, RFC1982-style sequence-number staleness/
  reordering rejection (correct under `uint32` wraparound), and packet-loss-percentage tracking
  (from sequence-number gaps) — zero ESP-IDF dependency by design, same driver-testing convention
  as Milestone 3's MPU6050 driver (see AGENTS.md). The design brief's central safety requirement —
  ESP-NOW's receive callback runs in the Wi-Fi driver's own task context and must do minimal work
  only — is honored by `esp_now_recv_cb()` (bounds-check + non-blocking queue post, no parsing/
  logging/blocking) with all real parsing/validation/sequence-tracking work moved into
  `esp_now_radio_process_pending()`, called once per cycle from Milestone 6's existing `RadioTask`
  (`firmware/main/radio_task.c`) rather than a new task. Radio-loss detection is exposed through
  `radio_health_t.link_alive` (a configurable staleness timeout,
  `CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS`, default 500ms) — Milestone 16 decides what to do
  about it, this milestone only makes it reliably detectable. RSSI is real, not fabricated:
  ESP-IDF's `esp_now_recv_info_t.rx_ctrl->rssi` is populated by the Wi-Fi driver for every received
  frame (confirmed against this project's pinned ESP-IDF v5.5.5 headers), carried through to
  `radio_health_t.rssi_dbm`. Gated behind `CONFIG_BICOPTER_RADIO_ENABLED` (default off, no
  ground-station peer paired yet — same pattern as `CONFIG_BICOPTER_SENSORS_ENABLED`), configured
  via a new "Bicopter radio (ESP-NOW)" Kconfig menu (peer MAC, Wi-Fi channel, staleness timeout).
  `idf.py build` succeeds both with the option off (default) and on (a real build against
  ESP-IDF's actual `esp_wifi`/`esp_now`/`nvs_flash` libraries, not review alone — confirmed in this
  environment). `tests/radio_packet_test.c` (47 checks) covers command/telemetry packet
  round-trips, malformed/undersized/corrupted-packet rejection, non-finite-float rejection,
  sequence staleness/reordering/wraparound, packet-loss-percentage computation against
  hand-derived gap patterns, and staleness-timeout boundary arithmetic — everything host-testable.
  No physical ESP32 pair was available in this environment, so real over-the-air packet
  delivery, latency, loss, range, and RSSI values under actual RF conditions remain unverified —
  see [docs/radio.md](docs/radio.md)'s full verified-vs-deferred breakdown, including the full
  pairing procedure for whoever has real hardware.

- [x] **15. RC receiver abstraction**
  Deliverable: a second `Radio`-interface (or parallel input) implementation for a conventional
  RC receiver (PPM/SBUS/etc., protocol TBD), selectable alongside or instead of ESP-NOW.
  Done when: stick/channel input from a real RC receiver is decoded into the same setpoint
  representation ESP-NOW produces, on hardware.
  Done via: `firmware/components/radio/` — `crsf_frame.c/.h` (pure CRC8/frame-sync/decode/
  calibration logic, zero ESP-IDF dependency) and `crsf_radio.c/.h` (ESP-IDF UART driver
  plumbing), the second concrete `Radio` backend alongside Milestone 14's ESP-NOW. CRSF was chosen
  over SBUS primarily because it carries a real per-frame CRC8 (this milestone's explicit
  checksum/CRC requirement) — classic SBUS has no CRC/checksum field at all, a real protocol fact
  reported honestly rather than glossed over; see [docs/radio.md](docs/radio.md) for the full
  tradeoff writeup, including the finding that SBUS's usual inverted-UART objection doesn't
  actually hold on this project's ESP32 target (`uart_set_line_inverse()`). Uses ESP-IDF's UART
  driver in interrupt/ring-buffer mode (`uart_driver_install()`), not busy-polling; the
  ISR/driver-task/RadioTask-context split is fully documented in docs/radio.md, including how it
  differs from Milestone 14's own callback (here, ESP-IDF's own UART driver does the interrupt
  work, not code this project wrote). Channel-to-function mapping and endpoint/center calibration
  are both Kconfig-configurable, not hardcoded to one vendor's convention. `radio_is_stale()`
  (Milestone 14) is reused unchanged for staleness/`link_alive`; packet-loss is estimated from
  elapsed time against a configured nominal frame period, since CRSF carries no sequence number
  (a coarser, honestly-documented approximation of ESP-NOW's exact sequence-gap figure).
  `radio.h` required no changes — confirmed in docs/radio.md, along with the two `radio_command_t`
  fields (sequence, timestamp_us) this backend fills differently due to data CRSF's wire format
  doesn't carry. `firmware/main/radio_task.c` wires CRSF in as a build-time either/or alternative
  to ESP-NOW, gated behind a new `#error` guard that fails the build if both
  `CONFIG_BICOPTER_RADIO_ENABLED` and `CONFIG_BICOPTER_CRSF_RADIO_ENABLED` are set — confirmed to
  actually fire in this environment, not just written and assumed correct. `idf.py build` succeeds
  with the option off (default), on, and confirmed to fail loudly (not silently pick one) with both
  radio options on at once. `tests/crsf_frame_test.c` (140 passing checks) covers CRC8 against the
  published CRC-8/DVB-S2 check value, RC_CHANNELS_PACKED decoding (including a distinct-value
  vector to catch bit-packing offset errors), rejection of corrupted/malformed/wrong-type frames,
  the frame synchronizer's resync behavior, channel-to-command calibration/clamping, and the
  packet-loss estimate. No physical CRSF transmitter/receiver was available in this environment —
  real channel values/ranges, frame timing, and UART signal integrity remain unverified; see
  docs/radio.md's full verified-vs-deferred breakdown and calibration procedure for whoever has
  real hardware.

- [x] **16. Safety / failsafes**
  Deliverable: `flight_core/safety/` + `firmware/components/safety/` — arming logic, signal-loss
  and sensor-fault failsafes, battery-voltage cutoffs, and the SafetyTask behavior.
  Done when: each failsafe condition (radio loss, sensor fault, low battery, disarm) is
  exercised in the simulator and/or on the bench and produces the correct safe response.
  Done via: `firmware/components/safety/` (`flight_mode.c/.h`, `arming.c/.h`, `failsafe.c/.h`,
  `actuator_command_check.c/.h`) — plain, ESP-IDF-free C, same driver-testing-convention split as
  every other sensor/actuator/radio driver, so `firmware/main/safety_task.c` (also C) calls it
  directly with no C++/ESP-IDF-component bridge to build; `flight_core/safety/` stays an empty
  scaffold (same status `flight_core/dynamics/` has held since Milestone 1) — see
  [docs/safety.md](docs/safety.md)'s "Why firmware/components/safety/, not flight_core/safety/"
  section for the full reasoning. The full `BOOT`/`DISARMED`/`ARMED`/`STABILIZE`/`ALTITUDE_HOLD`
  (Kconfig-gated off by default)/`FAILSAFE`/`ERROR` state machine is real and wired into
  `SafetyTask`'s existing 100Hz loop, replacing Milestone 6's always-disarmed stub — `FAILSAFE` is
  latching (only clears via explicit disarm, never auto-resumes flight) and `ERROR` is reachable
  only from `BOOT`/`DISARMED` (in-flight faults always route through `FAILSAFE` instead, a single
  well-tested in-flight fault path, not two). Arming requires every documented precondition (valid
  IMU/estimator, live radio + explicit arm command, battery above LOW, no critical errors, and the
  design brief's explicit stationary-during-arming check) with no partial-credit path — the system
  never auto-arms. Every required failsafe condition except task-watchdog failure (not
  representable at the application level — see docs/safety.md) is detected with a configurable,
  per-condition response defaulting to a safe motor shutdown/disarm (never an attempted autonomous
  landing, per the design brief), with `FAILSAFE_RESPONSE_LANDING` left as a documented,
  unimplemented extension point; radio loss specifically fires deterministically off
  `radio_health_t.link_alive` alone. Battery monitoring is new in
  `firmware/components/power/` — ADC-based (ESP32's line-fitting calibration scheme, its only
  supported scheme), a configurable voltage-divider ratio (no divider chosen yet — see
  docs/hardware.md), an explicitly-approximate voltage-to-percentage curve, configurable LOW/
  CRITICAL thresholds, and optional current sensing (supported, not required, per this milestone's
  brief) — gated behind `CONFIG_BICOPTER_BATTERY_ENABLED` (default off, no battery hardware chosen
  yet), failing closed (blocks arming) while disabled. Real live-data wiring this milestone: IMU/
  barometer validity and raw gyro rate (`SafetyTask` peeks `SensorTask`'s sample queue), radio arm
  command + link health (`RadioTask` now publishes via the new `radio_state.h`, the first live
  cross-task consumer of its decoded output), and battery voltage/percent/thresholds when enabled.
  Honestly not wired: estimator validity and attitude, since no estimator runs in firmware yet
  (`EstimatorTask` still only logs/discards — same open item as Milestones 8/10-13) —
  `estimator_valid` is hardcoded false, never faked true, so arming is structurally blocked in
  today's firmware regardless of hardware state; and the invalid-actuator-command check, since
  `FlightControlTask` calls no real allocator yet. See docs/safety.md's "What's wired to live
  data, and what isn't yet" section for the full breakdown. `idf.py build` succeeds with
  `CONFIG_BICOPTER_BATTERY_ENABLED`/`CONFIG_BICOPTER_BATTERY_CURRENT_SENSE_ENABLED`/
  `CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED` each off (default) and on. 185 new passing host-side
  checks across `tests/flight_mode_test.c` (54), `tests/arming_test.c` (27),
  `tests/failsafe_test.c` (37), `tests/actuator_command_check_test.c` (12), and
  `tests/battery_convert_test.c` (55) cover the full transition table (valid and invalid
  transitions), every arming precondition individually blocking arming plus the all-clear case,
  every failsafe condition firing and not-firing, and the battery conversion math. No physical
  IMU/radio/battery hardware was available in this environment to exercise a real end-to-end arm/
  fly/failsafe cycle — see docs/safety.md for the full verified-vs-deferred breakdown.

- [ ] **17. Hardware integration**
  Deliverable: all of the above assembled and flown/tested on the finalized physical vehicle,
  including any wiring/mounting/calibration documentation needed to reproduce the build.
  Done when: the vehicle achieves stable controlled flight (or the tethered/bench equivalent
  agreed for first hardware tests).
  **Requires physical ESP32/MPU6050/BMP581/ESC/servo/battery hardware in hand.** Unlike every
  milestone completed so far (each buildable, testable, and honestly verified-vs-deferred entirely
  without hardware), this milestone's actual deliverable — flight, or a tethered/bench equivalent —
  cannot be produced, faked, or meaningfully simulated without the real vehicle in hand. It is
  explicitly a **captain-directed** milestone, not one a crewmate can complete without that
  hardware: it does not get dispatched until real hardware exists, and no amount of additional
  software work substitutes for it. [docs/bench_test.md](docs/bench_test.md) (Milestone 18)
  documents the exact bring-up sequence this milestone is meant to run against once that hardware
  is available.

- [x] **18. Bench-test tooling**
  Deliverable: `tests/` and/or standalone tooling for bench validation — motor/servo output
  checks, sensor calibration routines, log capture/analysis — usable without full flight.
  Done when: a documented bench procedure exists for validating each subsystem in isolation
  before flight.
  Done via: three build-time operating modes (`SIMULATION`/`HARDWARE_TEST`/`FLIGHT`, a Kconfig
  choice — see [docs/bench_test.md](docs/bench_test.md) for the build-time-vs-boot-time-latched
  safety rationale), with `esc_test` (motor spin) reachable only in a `HARDWARE_TEST` build and
  gated behind a literal, exact `CONFIRM` token distinct from the normal arm/throttle path. A new
  `BENCH_TEST` build configuration (`CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED`) makes
  `firmware/components/actuators/src/pwm_esc_output.c` — the only file that ever calls ESP-IDF's
  LEDC PWM API for a motor — `#if`-exclude every one of those calls entirely; verified directly by
  `nm`-inspecting the compiled object (zero `ledc_*` references in a BENCH_TEST build, all four
  present otherwise), not just documented. A new `firmware/components/bench_test/` component (a
  minimal custom UART command parser, chosen over ESP-IDF's `esp_console` so the safety-relevant
  parsing/confirmation logic stays plain, ESP-IDF-free, and host-tested per this project's driver-
  testing convention) adds a `BenchTestTask` UART console with four independently-usable commands:
  continuous sensor streaming, continuous radio-command streaming, direct single-servo angle
  commands, and the gated single-motor `esc_test` (throttle hard-clamped, auto-returns to idle
  after a configured duration, loudly logs a warning on every invocation). This milestone also
  wires `actuators_init_safe()` into `main()` for the first time, behind a new
  `CONFIG_BICOPTER_ACTUATORS_ENABLED` gate (default off, same "no board chosen yet" pattern every
  other optional-hardware option in this project uses) with placeholder GPIO/pulse-width Kconfig
  config, so the bench-test console's servo/esc_test commands have real actuator handles to
  command once a board exists. `idf.py build` succeeds for all three operating modes and for
  `BENCH_TEST` (combined with `HARDWARE_TEST`, the intended real-usage combination); 68 new
  passing host-side checks (`tests/bench_test_command_test.c`,
  `tests/pwm_esc_bench_test_gate_test.c`, `tests/pwm_esc_bench_test_gate_disabled_test.c`) cover
  the command parser (including every confirmation-token near-miss) and the BENCH_TEST gate-value
  translation under both configurations. See [docs/bench_test.md](docs/bench_test.md) for the full
  writeup, including the recommended real-hardware bring-up sequence (propellers off first; sensors
  before actuators; servos before motors; verify motor/servo direction and control signs; verify
  failsafe behavior; restrained/tethered first powered test) that Milestone 17 is meant to actually
  run against. No physical hardware was available in this environment — see docs/bench_test.md's
  verified-vs-deferred section.
