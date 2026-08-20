# Estimation

**Stub.** No estimator code exists yet. This document will be filled in when the state-estimator
milestone lands — the actual fusion algorithm (e.g. complementary filter, Madgwick/Mahony, or an
EKF) and its derivation belong there, not here.

## What will live here

As milestone 8 (see [TODO.md](../TODO.md)) lands, this document will cover:

- **Attitude estimation approach** — the specific algorithm chosen for fusing gyro (angular
  rate) and accelerometer (gravity-vector reference) data into an attitude estimate, and why it
  was chosen over the alternatives, including how it's kept independent of which IMU part is
  fitted (see [hardware.md](hardware.md) — `Imu` is a part-agnostic interface).
- **Altitude/vertical-velocity estimation approach** — how barometer pressure readings (via the
  part-agnostic `Barometer` interface) are fused with accelerometer data for a smoother, faster
  altitude and vertical-speed estimate than the barometer alone provides.
- **Attitude representation** — the concrete choice (quaternion, rotation matrix, or Euler)
  used internally by the estimator and exposed to the controllers, and how it maps onto the
  body-frame convention fixed in [README.md](../README.md).
- **Calibration** — any sensor calibration (gyro bias, accelerometer scale/offset) the estimator
  depends on, and where that calibration data is expected to come from (factory constant vs.
  runtime/bench calibration routine — the latter likely ties into milestone 18's bench-test
  tooling).

## Interim conventions fixed now

- The estimator consumes only the `Imu` and `Barometer` hardware-abstraction interfaces from
  [architecture.md](architecture.md), never a specific part's driver directly, so it runs
  unmodified against both real hardware and the simulator's synthetic sensor implementations.
- All estimator inputs and outputs use the SI units and body-frame convention from
  [README.md](../README.md).
