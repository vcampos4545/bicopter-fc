# Math

`flight_core/math/` (Milestone 7) is the platform-independent vector/matrix/quaternion library
every later `flight_core` milestone (estimation, control, dynamics) builds on. It has zero
ESP-IDF/OS dependency — pure, portable C++17 — and is exercised by `firmware/` and (from
Milestone 9 on) `simulator/` unmodified, per the one-`flight_core`-two-runtimes design in
[architecture.md](architecture.md).

This document fixes the conventions used throughout: get these wrong (or leave them
undocumented) and every later milestone that touches attitude silently inherits the bug. See
[README.md](../README.md#coordinate-frame-and-units-convention) for the body-frame/units
convention this reconciles with.

## Scope

`flight_core/math/` is deliberately small — not a general-purpose linear-algebra library:

- `Vec3` — a 3D vector (`add`/`subtract`/`scale`/`dot`/`cross`/`normalize`).
- `Mat3` — a fixed 3x3 matrix (`matrix*vector`, `matrix*matrix`, `transpose`), sized for exactly
  what a small-body-rate flight controller needs: rotation matrices and the inertia-tensor
  `I * omega` computation. No NxN support, no decomposition beyond what quaternion conversion
  needs.
- `Quaternion` — identity, Hamilton-product multiplication, conjugate/inverse, normalization,
  body-rate integration, and Euler-angle/rotation-matrix conversion.

All three use `float` (single precision), matching the ESP32's single-precision hardware FPU —
the same numeric type the estimator and control milestones will run on-target.

## Quaternion convention

- **Hamilton product**, right-handed, **scalar-first storage `[w, x, y, z]`**
  (`flight_core/math/include/quaternion.h`). This is the convention used by most robotics/
  aerospace references (e.g. Solà, "Quaternion kinematics for the error-state Kalman filter") and
  by Eigen — picked so the estimation milestone can cross-check its math against that literature
  without a sign/order translation step.
- **A `Quaternion` represents the rotation from body frame to world frame (NED)**: for a vector
  `v_body` expressed in body-frame coordinates, `q.rotate(v_body)` (equivalently
  `q.toRotationMatrix() * v_body`) gives the same physical vector expressed in world-frame
  coordinates. This is the standard "attitude quaternion" convention — it's the orientation of
  the body relative to the world, which is what an estimator publishes and a controller consumes.
- **Body-frame angular velocity integrates on the right**: `q(t+dt) = normalize(q(t) *
  dq(omega_body, dt))`. Gyro measurements are in body-frame coordinates, and right-multiplication
  is exactly the composition rule for "apply this rotation in the frame the quaternion's own axes
  currently point along" — i.e. body frame. (Left-multiplying would instead apply the increment
  in world-frame coordinates, which is not what a gyro reading means.)

### Why the exponential-map integrator, not the linearized `q_dot = 0.5*q*omega_quat` step

`Quaternion::integrate(omega, dt)` (`flight_core/math/src/quaternion.cpp`) uses the exact
exponential-map update for constant angular velocity over the interval:

```
theta = |omega| * dt
dq    = [cos(theta/2), axis*sin(theta/2)]   where axis = omega / |omega|
q(t+dt) = normalize(q(t) * dq)
```

rather than the first-order linearization `q_dot = 0.5 * q * omega_quat` (`omega_quat = [0,
omega]`) integrated with a single Euler step. The exponential-map form is *exact* for a constant
angular rate over `dt`, regardless of step size — the linearized step's error grows with
`(|omega|*dt)^2` and would otherwise force either a smaller step or a correction term to stay
accurate at this project's `SensorTask`/`EstimatorTask` cadence (2 ms period, see
[architecture.md](architecture.md#freertos-tasks)) under aggressive body rates. The extra
`sin`/`cos` cost per call is a good trade at that call rate on an ESP32. Below a small-angle
threshold (`|omega|*dt < 1e-6` rad), `integrate()` falls back to the first-order Taylor expansion
of the same formula instead of normalizing a near-zero rotation axis — see the `kSmallAngle`
comment in `quaternion.cpp`.

`tests/quaternion_test.cpp`'s `test_integrate_constant_angular_velocity` is the property test
that pins this down: integrating a constant `omega` for a known total time, in either one large
step or many small steps, produces the same expected rotation.

### Euler sequence: ZYX (yaw-pitch-roll), matching README's body axes

`EulerAnglesZYX` / `Quaternion::FromEulerZYX` / `Quaternion::toEulerZYX` use the standard
aerospace **3-2-1 (ZYX) intrinsic sequence**: yaw first (about world Z), then pitch (about the
new Y), then roll (about the new X) — i.e. the rotation matrix composes as `R = Rz(yaw) *
Ry(pitch) * Rx(roll)`. This is not an arbitrary choice: it's the sequence whose individual axes
line up with README.md's body-frame convention (`roll` about X, `pitch` about Y, `yaw` about Z,
right-handed, X-forward/Y-right/Z-down), so "roll" in this library's Euler output means exactly
the same physical rotation as "roll" everywhere else in this repository's docs.

**Gimbal lock:** at `pitch = +-90 deg` the ZYX sequence loses a degree of freedom — only
`roll - yaw` (at +90) or `roll + yaw` (at -90) is determined by the rotation, not roll and yaw
individually. The naive `atan2`-based formulas divide two quantities that both vanish at the
singularity, so a direct implementation returns float-rounding noise there, not a graceful
answer. `Quaternion::toEulerZYX()` special-cases `|sin(pitch)|` above a `1 - 1e-6` threshold and
uses the documented convention **roll = 0**, folding the entire remaining rotation into yaw
(derived directly from the quaternion's `w`/`x` components — see the comment above the branch in
`quaternion.cpp`). `tests/quaternion_test.cpp`'s `test_euler_gimbal_lock` locks this behavior down
and checks that the recovered (roll=0, pitch=+-90, combined-yaw) angles reconstruct the exact same
physical rotation as the original input, even though the individual roll/yaw split differs.
Ordinary (non-gimbal-locked) attitudes round-trip through Euler angles to within float tolerance;
see `test_euler_round_trip`.

### Rotation matrix conversion

`Quaternion::toRotationMatrix()` / `Quaternion::FromRotationMatrix()` use the standard Hamilton
quaternion<->rotation-matrix formulas, consistent with the body-to-world convention above (`R *
v_body == v_world`). `FromRotationMatrix()` uses Shepperd's method — branching on the trace and
the three diagonal entries to pick whichever of `w`/`x`/`y`/`z` is best-conditioned to divide by,
rather than a single formula that's numerically unstable near some attitudes.

## Vector/matrix scope notes

- `Vec3::normalized()` / `Quaternion::normalized()` return a documented safe fallback (`Zero()` /
  `Identity()`) instead of dividing by ~0 when the input's length/norm is below a small epsilon —
  see the `kMinNormalizableLength` / `kMinNormalizableNorm` comments in the respective `.cpp`
  files. Callers in the estimator/control milestones should not need their own zero-guard before
  normalizing.
- `Mat3` defaults to the **zero** matrix (consistent with `Vec3`'s zero default), not identity —
  callers that want a rotation/identity matrix ask for `Mat3::Identity()` explicitly. This avoids
  a silent "uninitialized-looking" identity appearing where a zero accumulator was intended (e.g.
  building up a sum of outer products).
- Row-major storage; `Mat3::FromDiagonal()` exists specifically for constructing a diagonal
  inertia tensor, the motivating `I * omega` use case for the estimation/control milestones.

## Build and test

`flight_core/CMakeLists.txt` builds `flight_core` as a plain static library (no ESP-IDF
dependency) — see [README.md](../README.md#flight_core-platform-independent-library) for the
standalone build commands. `tests/CMakeLists.txt` pulls that library in via `add_subdirectory`
and links `tests/vec3_test.cpp`, `tests/mat3_test.cpp`, and `tests/quaternion_test.cpp` against
it directly (unlike the firmware-driver tests, which compile ESP-IDF-free `.c` sources by
relative path per [AGENTS.md](../AGENTS.md#driver-testing-convention) — `flight_core` is already
a real standalone CMake target, so its tests link the library rather than re-compiling its
sources). All three suites run under `ctest` alongside the existing driver/task tests; see
[README.md](../README.md#tests) for the full `cmake`/`ctest` invocation.
