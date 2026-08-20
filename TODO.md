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

- [ ] **3. MPU6050 driver + tests**
  Deliverable: an ESP-IDF component driving the MPU6050 over I2C (init, calibration read,
  accel/gyro sample retrieval) behind the `Imu` hardware-abstraction interface, with unit tests.
  Done when: the driver reads real accel/gyro samples on hardware and has test coverage for its
  parsing/scaling logic independent of the bus.

- [ ] **4. Barometer driver**
  Deliverable: an ESP-IDF component driving the barometric pressure sensor (BMP390 or BMP581 —
  whichever is fitted) behind the `Barometer` interface, without coupling any downstream code
  to the specific part.
  Done when: the driver reads pressure/temperature on hardware and altitude derivation is
  covered by a unit test independent of the bus.

- [ ] **5. ESC + servo hardware abstraction**
  Deliverable: `MotorOutput` and `ServoOutput` implementations driving the BLHeli ESCs and tilt
  servos (PWM/DShot as appropriate), with configurable protocol/range/calibration.
  Done when: commanded values on hardware produce correct, safely-bounded ESC/servo output,
  verified on a bench (props off).

- [ ] **6. FreeRTOS task architecture**
  Deliverable: the task skeleton described in `docs/architecture.md` (SensorTask, EstimatorTask,
  FlightControlTask, RadioTask, TelemetryTask, SafetyTask) wired up with their target
  rates/priorities and the queues/synchronization between them, running with stub payloads.
  Done when: all tasks run concurrently at their target rates without missed deadlines or
  priority inversions, measured on hardware.

- [ ] **7. Math library**
  Deliverable: `flight_core/math/` — vectors, quaternions/rotations, and any numerical utilities
  the estimator and controllers need, platform-independent and unit-tested.
  Done when: the library has test coverage for rotation composition, normalization, and the
  operations later milestones depend on.

- [ ] **8. State estimator**
  Deliverable: `flight_core/estimation/` — an attitude (and altitude) estimator fusing IMU and
  barometer data into the vehicle state, per the approach chosen and documented in
  `docs/estimation.md`.
  Done when: the estimator converges to a correct attitude estimate against recorded or
  simulated sensor data, with unit tests.

- [ ] **9. Bicopter dynamics simulator**
  Deliverable: `simulator/physics/` — a rigid-body dynamics model of the bicopter (mass,
  inertia, motor thrust/torque, servo tilt geometry) driving simulated `Imu`/`Barometer`
  implementations that `flight_core` reads through the same HAL interfaces used on hardware.
  Done when: the simulator runs an open-loop physics integration that responds plausibly to
  fixed motor/servo commands (e.g. correct free-fall and torque response), independent of any
  controller.

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
