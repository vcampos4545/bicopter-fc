# Control

## Status

Milestone 10 (this document's first real content) implements the generic `Pid` building block
and the `RateController` it's used to build, both in `flight_core/control/`. Milestone 11 (the
attitude controller, attitude error -> rate setpoint) and Milestone 12 (control allocation, body
torque/thrust -> per-motor throttle and per-servo tilt angle) remain unimplemented — see
[TODO.md](../TODO.md).

All control quantities use the body-frame and units convention from [README.md](../README.md):
SI units throughout, right-handed body frame with X=forward, Y=right, Z=down. Control allocation
(Milestone 12) will express its output through the `MotorOutput`/`ServoOutput` hardware-
abstraction interfaces described in [architecture.md](architecture.md), so the same allocation
code runs unmodified on `firmware/` and in `simulator/` — that remains true of this milestone's
code too: `flight_core/control/` has zero ESP-IDF dependency (per [AGENTS.md](../AGENTS.md)'s
`flight_core` rule) and is exercised identically by `tests/` today and by the simulator once
Milestone 13 closes the loop.

## `Pid`: the generic building block

`flight_core/control/include/pid.h` / `src/pid.cpp`. A single-axis PID controller with:

- Configurable `kp`/`ki`/`kd` gains (`PidConfig`) — no gain is a hardcoded constant inside `Pid`'s
  logic; every controller that uses `Pid` supplies its own `PidConfig`.
- Configurable output saturation (`output_min`/`output_max`).
- Explicit per-call `dt` (`update(setpoint, measurement, dt)`) — `Pid` never assumes a fixed loop
  rate. The same `Pid` type backs both the rate loop below (this milestone) and the future
  attitude loop (Milestone 11), and those two loops are not guaranteed to run at the same rate, so
  baking a rate into `Pid` itself would break the second use case.

### Anti-windup: clamping (conditional integration)

When a `Pid`'s output would saturate, integrating the current step's error further in the
direction that's already saturating the output serves no purpose — the output can't move past the
limit anyway, so that integral growth is pure windup that has to be unwound later, delaying
recovery once the error reverses sign.

`Pid::update()` handles this by clamping (a.k.a. conditional integration): each call tentatively
integrates the new error, then checks whether the resulting *unsaturated* output would exceed the
configured limits in the same direction the error is already pushing. If so, the tentative
integral is discarded and the integral term is held at its previous value for that step instead
(it remains free to shrink back once the error reverses or the output stops saturating). If not,
the tentative integral is committed normally.

This was chosen over the other common scheme, back-calculation (feeding a scaled measure of the
saturation error back into the integrator via an extra tracking-time-constant gain), because
clamping needs no additional tuning parameter beyond the `output_min`/`output_max` limits `Pid`
already requires for saturation itself. For a generic, reusable building block meant to be
configured per-axis and per-loop by several different callers, keeping the anti-windup scheme's
config surface to "just the saturation limits you were already going to set" was judged more
valuable than back-calculation's typically-faster unwind, which would need its own gain tuned per
use site. `tests/pid_test.cpp`'s `test_anti_windup_on_saturation` exercises this directly: it
saturates the output, confirms the integral stops growing while saturated, and confirms the output
leaves the saturation ceiling on the very next step once the error reverses (rather than being
stuck there for many steps unwinding accumulated windup).

### Derivative convention: derivative-on-measurement

`PidConfig::derivative_on_measurement` (default `true`) selects differentiating the *measurement*
rather than the *error* (`setpoint - measurement`). A step change in setpoint makes the error jump
discontinuously; differentiating that jump produces a large, physically meaningless output spike
known as "derivative kick." The measurement itself (e.g. the estimator's current body rate) never
jumps discontinuously the way a setpoint can, so differentiating it stays smooth across setpoint
changes while still reacting to how the measured quantity is actually moving. Derivative-on-error
remains available (`derivative_on_measurement = false`) for callers that want it; both conventions
produce numerically identical output when the setpoint happens to be held constant, and differ
only in behavior right at a setpoint step — `tests/pid_test.cpp`'s `test_derivative_response`
demonstrates both, including the kick derivative-on-error produces that derivative-on-measurement
does not.

### What `dt <= 0` does

`update()` treats a non-positive `dt` (a caller bug, a clock glitch, or the very first call before
a real interval exists) as "skip integration and differentiation this call": it returns the
clamped proportional-only response rather than dividing by zero or by a negative number. This
never corrupts the integral/derivative history — `dt <= 0` calls don't update
`previous_measurement_`/`previous_error_`, so the next call with a valid `dt` still differentiates
against the last real sample.

## `RateController`: the rate (angular-velocity) control loop

`flight_core/control/include/rate_controller.h` / `src/rate_controller.cpp`. Three independent
`Pid` instances, one per body axis (roll rate, pitch rate, yaw rate), with **zero cross-coupling
between axes inside the controller itself** — roll-rate error feeds only the roll `Pid`, and so on.
Any cross-axis coupling a real bicopter exhibits (e.g. yaw reaction torque perturbing roll/pitch
through the rigid-body dynamics) is a property of the vehicle physics
(`simulator/physics/`'s `I*omega_dot + omega x (I*omega) = tau`, see
[dynamics.md](dynamics.md)), not something `RateController` compensates for — that's a future
control-design decision (e.g. adding explicit decoupling terms), not an accident of this
milestone's scope.

**Interface:**

```cpp
Vec3 update(const Vec3& desired_radps, const Vec3& current_radps, float dt);
```

- `desired_radps` / `current_radps`: body-frame angular velocity, rad/s, `x` = roll rate, `y` =
  pitch rate, `z` = yaw rate — `current_radps` is meant to come directly from
  `flight_core/estimation/`'s `AttitudeEstimate::angular_velocity_radps` (Milestone 8).
  `desired_radps` is a rate setpoint; Milestone 11's attitude controller is what will produce it
  from an attitude error, but nothing stops a caller from supplying one directly (e.g. a bench
  test, or a future rate-only flight mode).
- `dt`: seconds since the previous `update()` call, forwarded unchanged to each axis's `Pid` — see
  `Pid`'s own `dt` handling above.
- Returns the desired body torque, N·m, `x`/`y`/`z` = roll/pitch/yaw torque. This is **not** yet a
  motor/servo command — Milestone 12's control allocation is what turns a torque (plus a separate
  thrust demand) into per-motor throttle and per-servo tilt angle.

`tests/rate_controller_test.cpp`'s `test_axis_independence` is the direct acceptance-criteria
check: an isolated roll-rate error (desired/current differ only on the roll axis) produces
nonzero roll torque and ~zero pitch/yaw torque, and likewise for pitch and yaw in isolation.
`test_matches_standalone_pid_per_axis` further confirms each axis's output across several steps
(exercising integral/derivative history, not just a single call) matches an independently-driven
`Pid` configured identically — i.e. `RateController` really is "three independent `Pid`s," not a
reimplementation that happens to look similar.

## Gains are placeholders, not tuned values

`RateControllerConfig`'s default `PidConfig`s (`flight_core/control/include/rate_controller.h`'s
`detail::DefaultRateAxisGains()`) are explicitly labeled placeholders — a starting point that
produces bounded, well-behaved output in `tests/`, nothing more. They are **not** tuned against
this vehicle, because no closed-loop simulation (Milestone 13, `flight_core`'s estimator + control
running against `simulator/physics/`'s dynamics model) or real hardware (Milestone 17) exists yet
to tune against — tuning gains without a real or simulated plant to tune them on would just be
inventing numbers. Real tuning is deferred to whichever of those two lands first for this vehicle.

This does not mean the gains are hardcoded: `PidConfig` and `RateControllerConfig` are ordinary,
publicly-settable config structs. Any caller (a test, the simulator, eventually firmware) supplies
its own values; the placeholder defaults exist only so `RateController()`'s default constructor
produces something usable for tests today, not as an assertion that these numbers belong on a real
or simulated vehicle.
