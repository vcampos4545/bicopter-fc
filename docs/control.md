# Control

## Status

Milestone 10 implements the generic `Pid` building block and the `RateController` it's used to
build. Milestone 11 implements `AttitudeController`, the outer (attitude) loop that converts an
attitude error into a rate setpoint for `RateController`. Milestone 12 implements
`ControlAllocator`, converting a desired total thrust and body torque into the vehicle's 4 motor/
servo commands — the full derivation, its small-angle-linearization simplification, the
controllability-near-hover findings, and the saturation/prioritization policy are in
[control_allocation.md](control_allocation.md), not repeated here. Nothing in `firmware/`/
`simulator/` calls any of `Pid`/`RateController`/`AttitudeController`/`ControlAllocator` yet —
wiring the full cascade together against real (or simulated) state is Milestone 13's job — see
[TODO.md](../TODO.md).

All control quantities use the body-frame and units convention from [README.md](../README.md):
SI units throughout, right-handed body frame with X=forward, Y=right, Z=down. Milestone 12's
`ControlAllocator` expresses its output in the exact normalized units the `MotorOutput`/
`ServoOutput` hardware-abstraction interfaces described in [architecture.md](architecture.md)
expect, so the same allocation code will run unmodified on `firmware/` and in `simulator/` once
Milestone 13 wires it to either — that remains true of this milestone's code too:
`flight_core/control/` has zero ESP-IDF dependency (per [AGENTS.md](../AGENTS.md)'s `flight_core`
rule) and is exercised identically by `tests/` today and by the simulator once Milestone 13 closes
the loop.

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
or simulated vehicle. `AttitudeControllerConfig` below follows the identical convention.

## `AttitudeController`: the attitude (angle) control loop

`flight_core/control/include/attitude_controller.h` / `src/attitude_controller.cpp`. The outer
loop of the cascaded architecture `docs/architecture.md` described from Milestone 1 onward:

```
desired attitude (setpoint) --> AttitudeController --> desired body rate --> RateController --> desired body torque
```

`AttitudeController` consumes the desired and current attitude *quaternions* (not Euler angles —
this repository's estimator and math library both work in quaternions, per
[math.md](math.md)/[estimation.md](estimation.md), and quaternion error avoids Euler angles'
gimbal-lock and ambiguous-sign issues near +-90 degree pitch) and produces a `Vec3` body rate,
consumed unchanged by `RateController::update()`'s `desired_radps` argument. Unlike `Pid`/
`RateController`, `AttitudeController` is **stateless** — a pure proportional map with no integral
or derivative term, so there is no `reset()`.

**Interface:**

```cpp
Vec3 update(const Quaternion& desired, const Quaternion& current,
            const Vec3& feedforward_radps = Vec3::Zero()) const;
```

- `desired` / `current`: body-to-world (NED) attitude quaternions (see [math.md](math.md)).
  `current` is meant to come directly from `flight_core/estimation/`'s
  `AttitudeEstimate::orientation` (Milestone 8).
- `feedforward_radps`: an optional additive desired body rate, `Vec3::Zero()` by default. Added
  to the proportional correction before the rate limit is applied. No caller in this repository
  supplies a nonzero value yet (there is no trajectory/rate-setpoint source ahead of this loop)
  — it exists because a pure attitude-hold P-loop is a well-known special case of a more general
  "track a moving attitude setpoint" loop, and adding the parameter now costs nothing while
  saving an interface change later if Milestone 13+ ever wants to command a *rate* in addition to
  an *attitude* (e.g. a coordinated turn or a scripted maneuver). If no real caller ever needs
  this, a future milestone finding it genuinely unused should feel free to remove it rather than
  treat this note as justification for keeping dead surface area.
- Returns the desired body rate, rad/s, `x`/`y`/`z` = roll/pitch/yaw rate — same shape as
  `RateController::update()`'s `desired_radps` parameter.

### Quaternion error convention: `q_error = current.inverse() * desired`

This is the single most important design decision in this milestone, because getting the
multiplication order backwards produces a controller that still compiles, still looks plausible
in isolated testing (it's still "proportional to the error," just with the wrong sign), and
silently commands the vehicle to rotate *away* from its setpoint — the classic failure mode this
kind of code is prone to.

The convention is derived directly from [math.md](math.md)'s already-fixed body-rate integration
rule, not invented independently: `Quaternion::integrate()` propagates orientation as
`q(t+dt) = normalize(q(t) * dq(omega_body, dt))` — a small rotation `dq`, expressed in the
*current* body frame's own axes, applied by right-multiplication. That is exactly what a gyro
measurement (and, by the same token, a *commanded* body rate) means: "rotate about these axes, as
currently oriented." Requiring the attitude controller's error quaternion to mean the same thing
— i.e. requiring `current * q_error ~= desired` for a small step — and solving for `q_error` gives

```
q_error = current.inverse() * desired
```

directly (left-multiply both sides by `current.inverse()`). Using the opposite order,
`desired.inverse() * current`, computes the rotation from `desired` to `current` instead of from
`current` to `desired` — the negated vector part, and therefore the reversed control direction.
Concretely, for `current = Identity` and `desired` a small +0.1 rad roll rotation,
`current.inverse() * desired` has vector-part `x ~= +0.05` (correct: command a positive roll
rate, roll toward the positive setpoint) while `desired.inverse() * current` has `x ~= -0.05`
(wrong: commands rolling away from the setpoint). `tests/attitude_controller_test.cpp`'s
single-axis tests pin down the correct sign numerically for all three axes, not just assert "some
nonzero output."

### Small-angle proportional law

For a rotation of angle `theta` about a unit axis `n`, a quaternion's vector part is
`sin(theta/2)*n`, which is `~= (theta/2)*n` for small `theta`. So `q_error`'s vector part already
approximates half of the "rotation vector" `theta*n` (a vector whose direction is the axis to
rotate about and whose magnitude is the angle to rotate by) that reduces the attitude error.
Multiplying by 2 recovers that rotation vector, and applying a per-axis gain
(`AttitudeControllerConfig::kp`, x=roll/y=pitch/z=yaw, same axis-independent style
`RateController` already established) turns it into a commanded body rate:

```
commanded[i] = 2 * kp[i] * q_error.{x,y,z}[i]        (i = roll, pitch, yaw)
```

This is the standard "quaternion feedback" technique for rigid-body attitude control (see e.g.
Wie, Weiss & Arapostathis, "Quaternion Feedback Regulator for Spacecraft Eigenaxis Rotations,"
1989) in its simplified proportional-only form — no attempt is made here at the fuller
Lyapunov-based nonlinear control law those references also derive, since Milestone 10's `RateController`
already closes an inner loop around the rate this outer loop commands.

**Small-angle accuracy.** The `theta/2 ~= sin(theta/2)` approximation is exact at `theta = 0` and
degrades smoothly: at `theta = 90` degrees it under-reads the "ideal" linear response by about
10%, and as `theta` approaches 180 degrees the vector part saturates at magnitude 1 instead of
continuing to grow with `theta`, so the commanded rate's *direction* stays correct but its
*magnitude* is heavily under-read relative to a hypothetical unbounded-linear law. This is judged
acceptable rather than compensated for: a bicopter's controlled flight envelope is attitude
stabilization near level flight, where attitude errors large enough for this nonlinearity to
matter (many tens of degrees) already indicate a failure condition `SafetyTask` (Milestone 16)
should be handling, not a regime this loop needs to be quantitatively accurate in.

**180-degree handling.** Quaternions have a double cover: `q` and `-q` represent the identical
physical rotation, but their vector parts have opposite sign. Before computing the vector part,
`AttitudeController::update()` negates all four components of `q_error` whenever `q_error.w < 0`.
This is the standard, minimal fix for that ambiguity — it guarantees the controller always picks
the representation whose rotation angle is in `[0, 180]` degrees, i.e. always commands the
*shorter* of the two directions that reach the same target attitude, rather than occasionally
locking onto the long way around for an error that happens to exceed 180 degrees.
`tests/attitude_controller_test.cpp`'s `test_shortest_path_beyond_180_degrees` exercises this
directly: a nominal +200 degree roll error produces a *negative* commanded roll rate (the shorter
-160 degree path), not a naive positive one.

This is explicitly **not** a dedicated large-angle or inverted-flight recovery strategy. It does
not resolve the genuine geometric singularity at exactly `theta = 180` degrees (at that exact
angle, rotating by 180 degrees about the computed axis or its exact opposite reaches the same
target attitude equally quickly — the "shorter" direction is genuinely ambiguous, not just
numerically ill-conditioned), and no special casing exists for a fully inverted vehicle beyond
what this one sign-fix already provides. This is judged acceptable for the same reason as the
small-angle limitation above: this bicopter's flight envelope is attitude-holding near level
flight, not aerobatics, so a dedicated recovery-from-full-inversion controller is out of scope
rather than a gap.

### Rate limit

`AttitudeControllerConfig::rate_limit_radps` is a per-axis symmetric clamp (rad/s) applied to the
total commanded rate (proportional correction + `feedforward_radps`) before it's returned — this
is what stops a large attitude error from asking `RateController` for a rate the vehicle has no
realistic hope of achieving or safely arresting. It's always applied (same "always-on saturation"
philosophy as `Pid::output_min`/`output_max`), with a non-positive value on an axis treated as "no
limit configured" for that axis rather than "clamp to zero" — this lets tests (and any future
caller with a genuine reason not to limit) disable it explicitly rather than needing a separate
enable flag. An independent per-axis clamp was chosen over clamping the combined rate vector's
magnitude for the same reason `RateController` keeps its three axes independent: it keeps the
config surface and the reasoning about any one axis's behavior simple, at the cost of not
preserving the *direction* of a saturated multi-axis rate command exactly — judged an acceptable
tradeoff at this milestone, same as `Pid`'s own independent-axis saturation.
`tests/attitude_controller_test.cpp`'s `test_rate_limit_saturates` confirms the clamp is exact and
that saturating one axis leaves the other two unaffected.

### Gains and limits are placeholders, not tuned values

`AttitudeControllerConfig`'s defaults (`detail::DefaultAttitudeKp()`,
`detail::DefaultRateLimitRadps()` in `attitude_controller.h`) use the identical value on all three
axes, the same "not physically motivated, just a starting point that behaves sanely in tests"
placeholder approach `RateController`'s `detail::DefaultRateAxisGains()` used at Milestone 10 — see
that section above for the full reasoning. Real tuning remains deferred to Milestone 13's
closed-loop simulator or Milestone 17's real hardware.

## `ControlAllocator`: control allocation

`flight_core/control/include/control_allocator.h` / `src/control_allocator.cpp`. The final stage
of the cascaded architecture:

```
desired attitude --> AttitudeController --> desired rate --> RateController --> desired torque
                                                              (+ a desired total thrust, from a
                                                               future altitude/throttle source)
                                                                       |
                                                                       v
                                                    ControlAllocator --> motor1/2 throttle,
                                                                         motor1/2 tilt
```

`ControlAllocator::allocate(desired_thrust_n, desired_torque_nm)` converts `RateController`'s
torque output (plus a total-thrust scalar) into the vehicle's 4 actual actuator degrees of
freedom, in Milestone 5's `MotorOutput`/`ServoOutput` normalized units. This is the mathematical
inverse of Milestone 9's forward-dynamics model, derived term-by-term from the same geometry
`bicopter_dynamics.cpp` implements — **not** an independently-invented mixing matrix. The full
derivation (a small-angle linearization around hover, decoupled into a thrust/roll solve followed
by a tilt/pitch/yaw solve), the central finding that this vehicle's geometry has no pitch-torque
authority near hover unless `VehicleParams::center_of_mass_offset_m` has a nonzero vertical
component, and the documented saturation/prioritization policy (thrust preserved over roll
accuracy; tilt/pitch/yaw saturate independently) are all in
[control_allocation.md](control_allocation.md) — deliberately not duplicated here, since that
document's derivation needs to be read alongside `bicopter_dynamics.cpp`'s equations, not
`control.md`'s.

Like `AttitudeController`, `ControlAllocator` is stateless (a pure function of its
constructor-supplied `VehicleParams`/`ControlAllocatorConfig` and each call's inputs) — no
`reset()`. `tests/control_allocator_test.cpp` (144 checks) includes the milestone's most important
test: allocator output round-tripped through Milestone 9's real `computeStateDerivative()` to
confirm the derived inverse actually reproduces the requested thrust/torque, not just a
plausible-looking one.
