# Control allocation

Milestone 12 lands `flight_core/control/control_allocator.{h,cpp}`: `ControlAllocator`, which
converts a desired total thrust and body torque (Milestones 10-11's output shape) into the
vehicle's 4 actual actuator degrees of freedom — motor1 throttle, motor2 throttle, motor1 tilt,
motor2 tilt, in Milestone 5's `MotorOutput`/`ServoOutput` normalized units.

This is the mathematical inverse of Milestone 9's forward-dynamics model
(`simulator/physics/bicopter_dynamics.cpp`'s `computeStateDerivative()`/`computeMotorEffect()`).
Per this project's design brief, that inverse is **derived** below from the exact same geometry,
thrust-direction convention, and reaction-torque model [dynamics.md](dynamics.md) already
established and tests — not invented mixing coefficients that happen to move the vehicle. Every
equation in this document traces back to a specific line in `bicopter_dynamics.cpp`; where a
simplification is introduced (there is one: small-angle linearization around hover), it is stated
explicitly, along with its valid range and what happens outside it.

## Reuse, not re-derivation

`motorThrustDirectionBody()` — the exact function `bicopter_dynamics.cpp` uses, and
`tests/bicopter_dynamics_test.cpp`'s `test_thrust_direction` verifies — moved from
`simulator/physics/` to `flight_core/vehicle/include/motor_geometry.h` in this milestone
specifically so `ControlAllocator` could call the **same compiled function**, not a duplicate.
This was necessary, not optional: `simulator/physics/`'s `bicopter_physics` CMake target links
`flight_core`, never the reverse, so `flight_core/control/` cannot depend on
`simulator/physics/`. Moving the shared geometry helper down to `flight_core/vehicle/` (next to
`VehicleParams`, which Milestone 9 placed there for the identical reason — see
[dynamics.md](dynamics.md) and `vehicle_params.h`'s file header) is the same "shared code lives on
the side of the dependency edge everything can reach" pattern already established in this
codebase, not a new one invented for this milestone. `simulator/physics/include/bicopter_dynamics.h`
still re-exposes `motorThrustDirectionBody()` (via `#include`) so no existing caller changed.

`ControlAllocator` also consumes `VehicleParams`/`MotorParams` directly — the same struct
`bicopter_dynamics.cpp` reads, with zero fields re-declared or duplicated.

## Notation

Matches `control_allocator.cpp`'s comments exactly, for `i` in `{1, 2}`:

| Symbol | Meaning | Source |
|---|---|---|
| `y_i` | `MotorParams::arm_offset_y_m` | `VehicleParams` |
| `c` | `center_of_mass_offset_m` | `VehicleParams` |
| `d_i` | `y_i - c.y` | Y moment arm relative to the true CoM |
| `k_i` | `thrust_coefficient_n` | `VehicleParams` |
| `s_i` | `spin_direction` | `VehicleParams` |
| `q_i` | `torque_coefficient_nm_per_n` | `VehicleParams` |
| `thrust_i` | motor `i`'s commanded thrust, N | solved for below |
| `tilt_i` | motor `i`'s commanded tilt, rad | solved for below |

## Deriving the forward map (recap of `computeMotorEffect()`)

For each motor, `bicopter_dynamics.cpp` computes:

```
thrust_i    = k_i * throttle_i^2
thrust_dir_i(tilt_i) = (-sin(tilt_i), 0, -cos(tilt_i))          // motorThrustDirectionBody()
F_i         = thrust_i * thrust_dir_i(tilt_i)
r_i         = (0, y_i, 0) - c  = (-c.x, d_i, -c.z)
torque_i    = r_i x F_i  +  s_i * q_i * thrust_i * thrust_dir_i(tilt_i)
```

Summed over both motors: `T = thrust_1 + thrust_2` (total thrust along body -Z at zero tilt) and
`tau = torque_1 + torque_2` (roll/pitch/yaw). Control allocation needs `(thrust_1, thrust_2,
tilt_1, tilt_2)` given `(T, tau)` — 4 unknowns, 4 equations, but the equations are **nonlinear** in
`tilt_i` (through `sin`/`cos`) and every torque term is a *product* of a thrust and a tilt, so
there is no clean closed-form inverse of the full nonlinear system. Per the milestone brief, the
documented, justified simplification used here is **small-angle linearization around hover**.

### Small-angle linearization

For small `tilt_i`: `sin(tilt_i) ~= tilt_i`, `cos(tilt_i) ~= 1`, so:

```
thrust_dir_i ~= (-tilt_i, 0, -1)
F_i ~= (-thrust_i * tilt_i, 0, -thrust_i)         // Fx_i, Fy_i(=0), Fz_i
```

Expanding `r_i x F_i` with `r_i = (-c.x, d_i, -c.z)` and `F_i = (Fx_i, 0, Fz_i)` (note `Fy_i` is
*exactly* zero always — tilt is a rotation about body +Y, so the thrust vector never leaves the
body X-Z plane, for any tilt angle, linearized or not):

```
(r_i x F_i).x = d_i*Fz_i - (-c.z)*0        = d_i*Fz_i
(r_i x F_i).y = (-c.z)*Fx_i - (-c.x)*Fz_i  = -c.z*Fx_i + c.x*Fz_i
(r_i x F_i).z = (-c.x)*0 - d_i*Fx_i        = -d_i*Fx_i
```

and the reaction-torque term `s_i*q_i*thrust_i*thrust_dir_i` is `(-s_i*q_i*thrust_i*tilt_i, 0,
-s_i*q_i*thrust_i)` — also exactly zero on the Y axis, for the same reason (`thrust_dir_i.y` is
identically zero, tilted or not).

Substituting `Fz_i ~= -thrust_i`, `Fx_i ~= -thrust_i*tilt_i`, and summing over both motors:

```
T     = thrust_1 + thrust_2                                                          (exact)
tau_x = -d_1*thrust_1 - d_2*thrust_2                                                 (roll)
tau_y = c.z*(thrust_1*tilt_1 + thrust_2*tilt_2) - c.x*T                              (pitch)
tau_z = d_1*thrust_1*tilt_1 + d_2*thrust_2*tilt_2 - (s_1*q_1*thrust_1 + s_2*q_2*thrust_2)  (yaw)
```

`T` and `tau_x` involve only `thrust_1, thrust_2` — no tilt dependence survives the linearization
at all, because it enters only through `Fz_i ~= -thrust_i*cos(tilt_i) ~= -thrust_i`, a
second-order effect in `tilt_i` that's dropped. `tau_y` and `tau_z` are linear in `tilt_i` **once
`thrust_1, thrust_2` are already fixed** (they appear as `thrust_i * tilt_i` products, not
`tilt_i` alone). This structure is what makes the system solvable in two independent stages rather
than one coupled 4x4 solve.

One additional term is dropped as second-order: the reaction torque's small `-s_i*q_i*thrust_i*
tilt_i` contribution to roll. At zero tilt (hover trim) reaction torque is purely a yaw effect (see
`tests/bicopter_dynamics_test.cpp`'s `test_reaction_torque_yaw`); away from trim it leaks a small
amount into roll proportional to `q_i * tilt_i`, which is neglected here on the same "small angle,
second order" grounds as the rest of this section. This is accurate provided `q_i` isn't large
relative to the arm-offset-driven roll term (`d_i`) — true for any physically reasonable propeller
reaction-torque coefficient, which is typically one to two orders of magnitude smaller than a
motor's thrust-times-arm-length moment.

## Stage 1: thrust allocation solves `(T, tau_x) -> (thrust_1, thrust_2)`

```
thrust_1 + thrust_2          = T
-d_1*thrust_1 - d_2*thrust_2 = tau_x
```

A 2x2 linear system, solved exactly (no further approximation beyond the linearization above):

```
thrust_1 = (tau_x + d_2*T) / (d_2 - d_1)
thrust_2 = T - thrust_1
```

The determinant `d_2 - d_1` simplifies to `y_2 - y_1` — `center_of_mass_offset_m.y` cancels out of
the *split ratio* entirely (it still enters both equations individually through `d_i`, but not
their difference). `y_2 - y_1` is zero only if both motors sit at the same body-Y position, a
degenerate/invalid vehicle configuration; `control_allocator.cpp` guards this (see "Degenerate
geometry" below).

**Roll authority comes entirely from differential thrust in this linearization** — confirming
[dynamics.md](dynamics.md)'s "Roll... differential thrust" statement, and now to the precision of
showing there's no first-order tilt contribution to roll at all near hover.

## Stage 2: tilt allocation solves `(tau_y, tau_z) -> (tilt_1, tilt_2)`, given `thrust_1, thrust_2`

With `thrust_1, thrust_2` now fixed numbers (Stage 1's result, after the clamping in "Saturation
and prioritization" below), substitute `u_i = thrust_i * tilt_i` — linear in `tilt_i` since
`thrust_i` is now a constant:

```
c.z*(u_1 + u_2) = tau_y + c.x*T_actual                          (pitch, rearranged)
d_1*u_1 + d_2*u_2 = tau_z + (s_1*q_1*thrust_1 + s_2*q_2*thrust_2)   (yaw, rearranged)
```

(`T_actual = thrust_1 + thrust_2` after Stage 1's clamping — see below.) Another 2x2 linear
system, this time in `(u_1, u_2)`, with determinant `c.z*(d_2-d_1) = c.z*(y_2-y_1)`. Solved via
Cramer's rule in `control_allocator.cpp` when nonzero; `tilt_i = u_i / thrust_i` recovers the
tilt angles (with a zero-thrust guard: a motor commanded to zero thrust has no defined "ideal"
tilt, since tilting it changes nothing — `control_allocator.cpp` leaves `tilt_i = 0` in that case).

## Controllability near hover: the central finding of this milestone

**The determinant of the pitch/yaw system is `c.z * (y_2 - y_1)`.**
`VehicleParams::center_of_mass_offset_m` defaults to `Vec3::Zero()`, and no test or configuration
anywhere in this repository sets it to a nonzero value — meaning **`c.z` is zero in every
configuration this project currently exercises, which makes that determinant exactly zero.**

This is not a limitation of `ControlAllocator`'s math — it is a provable structural fact about
this vehicle's *geometry as currently parameterized*: with both motors positioned purely along the
body Y axis (`(0, y_i, 0)` before any CoM offset) and the tilt axis also body +Y, both `r_i x F_i`
and the reaction-torque term have **exactly zero Y-component for every possible thrust and tilt
command**, linearized or not (shown above — `(r_i x F_i).y` and the reaction torque's Y-component
both vanish identically whenever `c.x = c.z = 0`). Common-mode tilt of both motors (tilting them
together) does produce a net body-X *force* — it makes the vehicle translate forward/aft — but
zero net *moment* about Y, because both motors' moment arms `r_i` have no X or Z component to
cross that force against.

**Pitch-torque authority near hover requires a nonzero `center_of_mass_offset_m.z`** — a real,
physically meaningful vertical offset between the motor-mounting plane and the vehicle's true
center of mass (e.g. a battery/frame mass sitting below the arms, which is typical for a real
bicopter build). `ControlAllocator` uses this offset exactly when configured nonzero (the `c.z`
terms above are not invented — they're the literal `(r_i x F_i).y` contribution derived the same
way as every other term in this document) and correctly reports pitch as **unachievable** rather
than silently doing nothing when `c.z` is zero (see "Saturation and prioritization" below).
`tests/control_allocator_test.cpp`'s `test_pitch_unachievable_with_zero_com_offset` and
`test_pitch_achievable_with_com_offset` both exercise this directly, round-tripped through
Milestone 9's real forward model.

**This is a real, load-bearing finding for whoever finalizes this vehicle's physical geometry**
(Milestone 17): as currently parameterized, this bicopter has roll and yaw authority near hover
but no pitch authority at all unless the build gives it a nonzero vertical CoM/motor-plane offset.
That is a hardware/mechanical decision, not something this milestone's allocation code can or
should invent a workaround for — see [dynamics.md](dynamics.md)'s "What each control axis'
authority comes from" section, which now cross-references this finding.

**Yaw authority, by contrast, comes from differential tilt** (`u_1 = -u_2` in the pitch-degenerate
fallback below), not primarily from reaction torque: `thrust_1, thrust_2` are already fully
committed by Stage 1 (the roll/total-thrust solve), so by the time Stage 2 runs, reaction torque's
contribution (`s_i*q_i*thrust_i`) is a **fixed, known bias** to correct for (folded into `yaw_rhs`
above), not a controllable lever — tilt is the only remaining free variable. If `torque_coefficient_
nm_per_n` is configured nonzero and the two motors don't spin opposite directions (see
[dynamics.md](dynamics.md)'s "Spin direction: not assumed opposite"), that bias is nonzero even at
hover trim; `ControlAllocator` already accounts for and cancels it as part of the `yaw_rhs`
term — it does not need a separately-tuned "yaw trim" constant.

### Degenerate geometry: the pitch-unachievable fallback

When `|c.z * (y_2-y_1)|` is below a small numerical-robustness threshold (see "On the epsilon
thresholds" below), `control_allocator.cpp` does not divide by ~0. Instead:

1. It marks the output `saturated = true` if the requested pitch torque (adjusted for the known
   `c.x*T` bias) was nonzero — pitch was asked for and honestly reported as not delivered.
2. It solves yaw *alone* using pure differential response: `u_1 = -u_2 = yaw_rhs / (d_1 - d_2)`.
   This is the natural choice once pitch is off the table — it uses exactly the lever
   [dynamics.md](dynamics.md) already identifies for yaw (differential tilt) and doesn't waste
   authority on a common-mode component that (with `c.z ~= 0`) wouldn't affect pitch anyway.
3. If `y_1 - y_2` is *also* near zero (both motors coincident — a doubly-degenerate, physically
   invalid configuration), even that fallback's denominator vanishes. `control_allocator.cpp`
   holds `tilt_1 = tilt_2 = 0` in that case rather than producing NaN/Inf, and marks `saturated`
   if a nonzero yaw was requested. This case cannot occur for a valid two-motor vehicle
   (`y_1 != y_2` by construction) and exists purely as a never-crash guard, not an expected
   operating mode.

### On the epsilon thresholds

`control_allocator.cpp`'s `kGeometryEpsilon` (`1e-6`) and `kThrustEpsilon` (`1e-6`) are
**numerical-robustness thresholds, not physical mixing coefficients** — they decide when a linear
solve's denominator is too close to zero to trust (dividing by `1e-9` would produce a nonsense
tilt command that then gets clamped to the limit anyway, just noisily and unpredictably), not
anything about *how* the vehicle is commanded to move. Since the geometric quantities they guard
(`center_of_mass_offset_m`, `arm_offset_y_m` differences) default to exactly `Vec3::Zero()` /
distinct configured values, a real vehicle configuration essentially never lands in the ambiguous
"nonzero but numerically negligible" zone these thresholds are sized for.

## Saturation and prioritization

Actuator limits come from two places, both already established by earlier milestones:

- **Throttle limits**: `ControlAllocatorConfig::throttle_min`/`throttle_max`, normalized `[0,1]`,
  defaulting to the full range `MotorOutput::write()` (Milestone 5) accepts. This mirrors
  Milestone 5's `pwm_esc_convert_config_t::min_throttle`/`max_throttle` concept re-expressed as a
  `flight_core`-owned config, since `flight_core/control/` cannot depend on the firmware-only,
  ESP-IDF-adjacent struct that actually holds it on real hardware.
- **Tilt limits**: `VehicleParams::MotorParams::min_tilt_rad`/`max_tilt_rad` (Milestone 9), used
  directly — no separate config surface invented.

**Priority order, applied in `allocate()`, most-preserved first:**

1. **Motor throttle bounds** — hard hardware limits, never violated under any circumstances.
2. **Total thrust** (weight support) — preserved as closely as the throttle bounds allow. This is
   the standard, well-justified control-allocation choice: losing lift is a worse failure mode
   than losing some torque authority (a vehicle that can't produce enough thrust falls; a vehicle
   with degraded roll/pitch/yaw authority merely responds sluggishly or drifts).
3. **Roll torque accuracy** — sacrificed first when honoring it would require violating (1) or
   (2). Implementation: `thrust_1` is clamped to its own bounds first; `thrust_2` is then
   recomputed as `T - thrust_1` to preserve total thrust exactly *if that recomputed value is
   itself in bounds*. Only if `thrust_2` *also* needs clamping does total thrust itself end up
   compromised — which only happens when the requested `(thrust, roll)` combination sits outside
   the combined envelope of both motors, i.e. genuinely unachievable by any split.
4. **Pitch/yaw torque accuracy** — `tilt_1`/`tilt_2` are clamped to their own limits
   independently, after being solved from the (now-final, post-clamp) thrust values. There is no
   "preserve a total" step here analogous to thrust's, because there's no meaningful "total tilt"
   quantity — each servo's limit is a hard mechanical bound with no other actuator to trade
   against.

Every clamp point sets `AllocatedCommand::saturated = true` when it actually changes the value
(not merely when the input was already out of range and re-clamps to the same result), so a
caller/telemetry consumer can distinguish "the vehicle is being asked for more than it has" from a
routine in-envelope command. `tests/control_allocator_test.cpp`'s `test_saturation_*` tests
exercise excess thrust, negative thrust, extreme yaw, and extreme roll, confirming the output
always stays finite and within configured bounds even for wildly unachievable requests (never a
"garbage" command reaching the actuators).

## Accuracy and its limits

The small-angle linearization is exact at `tilt = 0` and degrades smoothly as tilt grows, in two
concrete ways:

- **`sin(tilt) ~= tilt`**: at 15 degrees (~0.26 rad) the error is about 1.2%; at 30 degrees, about
  4.5%.
- **`cos(tilt) ~= 1`**: at 15 degrees the true vertical-thrust factor is `cos(15deg) ~= 0.966`
  (a ~3.5% overestimate of each tilted motor's contribution to total thrust); at 30 degrees,
  `cos(30deg) ~= 0.866` (~13.4%).

Both errors are dropped by design (that's what "small-angle" means), so a large commanded tilt —
which mostly arises from a large *yaw* or *pitch* demand, since roll doesn't use tilt at all in
this derivation — produces a correspondingly larger gap between the requested and actually-achieved
torque/thrust. `tests/control_allocator_test.cpp`'s isolated-axis tests use modest torque demands
(tilts under ~10 degrees) and check agreement to a fraction of a percent; the combined
multi-axis round-trip test deliberately drives tilts to roughly 15 degrees and uses a
correspondingly looser (8%) tolerance, documented at the call site as reflecting this exact
limitation rather than a looser bar for correctness.

This is judged acceptable for the same reason [control.md](control.md)'s `AttitudeController`
accepts its own small-angle degradation: this vehicle's controlled flight envelope is
stabilization near hover, where the commanded tilts/torques needed are modest. A future milestone
wanting tighter accuracy at large commanded tilt has two documented options, neither implemented
here: (a) a correction pass that re-evaluates the *actual* nonlinear `motorThrustDirectionBody()`
at the linearized solution and does one Newton-style correction step, or (b) a small
fixed-iteration numerical solve of the full nonlinear system directly. Both are legitimate
extensions per the milestone brief; this milestone's linearized closed form is the one actually
implemented, chosen because it is exact at the hover trim point this vehicle spends most of its
time near, is analytically verifiable term-by-term against Milestone 9's equations (rather than an
iterative solve's convergence behavior being the only thing to test), and costs a fixed, tiny
amount of arithmetic per control cycle with no convergence/iteration-budget failure mode to reason
about at all — the "never silently return a garbage/unclamped result" requirement is satisfied
structurally (every branch is closed-form) rather than by needing an iteration-limit fallback path.

## Interface

```cpp
struct ControlAllocatorConfig {
    float throttle_min = 0.0f;
    float throttle_max = 1.0f;
};

struct AllocatedCommand {
    float motor1_throttle = 0.0f;
    float motor2_throttle = 0.0f;
    float motor1_tilt_rad = 0.0f;
    float motor2_tilt_rad = 0.0f;
    bool saturated = false;
};

class ControlAllocator {
public:
    explicit ControlAllocator(const VehicleParams& params,
                               const ControlAllocatorConfig& config = {});
    AllocatedCommand allocate(float desired_thrust_n, const Vec3& desired_torque_nm) const;
};
```

Stateless, like `AttitudeController` (Milestone 11) — a pure function of its constructor-supplied
config and each call's inputs, no `reset()`. `AllocatedCommand` is a distinct type from
`simulator/physics/`'s `ActuatorCommand` (identical field shapes and units, since both mirror
Milestone 5's `MotorOutput`/`ServoOutput` convention) rather than the same type, because
`flight_core/control/` cannot depend on `simulator/physics/` (same dependency-direction
constraint as `motorThrustDirectionBody()` above) — `tests/control_allocator_test.cpp`'s
round-trip checks construct a `simulator::ActuatorCommand` from an `AllocatedCommand`'s fields
explicitly at the call site.

Nothing calls `ControlAllocator` yet — wiring it (and `RateController`/`AttitudeController`) into
`FlightControlTask` and the simulator's closed loop is Milestone 13's job, per
[control.md](control.md)'s Status section and [TODO.md](../TODO.md).

## Tests

`tests/control_allocator_test.cpp` (144 checks):

- `test_hover_symmetric_zero_tilt` — thrust = weight, zero torque: symmetric throttles, ~zero
  tilt, round-tripped through the real forward model to confirm both thrust and torque match.
- `test_pure_roll_produces_differential_thrust` — differential throttle, ~zero tilt (roll doesn't
  use tilt in this derivation), round-tripped against the forward model.
- `test_pure_yaw_produces_differential_tilt` — symmetric throttle, opposite-sign tilt, round-tripped
  against the forward model.
- `test_pitch_unachievable_with_zero_com_offset` / `test_pitch_achievable_with_com_offset` — the
  central controllability finding above, both directions, both round-tripped.
- `test_round_trip_combined_thrust_roll_pitch_yaw` — **the single most important test in this
  milestone**: a combined, non-degenerate, multi-axis command with a nonzero CoM offset (so all
  three torque axes are simultaneously achievable), allocator output fed through Milestone 9's
  real `computeStateDerivative()`, confirming the achieved thrust/torque approximately reproduces
  the request — i.e. that this derivation is actually a correct inverse of the forward model, not
  merely plausible-looking.
- `test_saturation_excess_thrust_clamps_to_envelope`,
  `test_saturation_negative_thrust_clamps_to_zero`, `test_saturation_extreme_yaw_clamps_tilt`,
  `test_saturation_extreme_roll_clamps_and_never_produces_garbage` — the documented
  clamping/prioritization policy, each confirming `saturated == true` and every output field
  finite and within its configured bound.
- `test_never_produces_nan_or_inf_for_adversarial_inputs` — a sweep of extreme thrust/torque
  combinations, confirming the output is always finite and in-bounds.
- `test_degenerate_zero_arm_offset_falls_back_without_crashing` — the doubly-degenerate geometry
  guard, confirming a graceful, finite, `saturated = true` fallback rather than NaN.

## Build

No new CMake target: `control_allocator.cpp` is added to `flight_core/CMakeLists.txt`'s existing
`add_library(flight_core ...)` sources (same pattern as `attitude_controller.cpp`).
`tests/control_allocator_test.cpp` links `bicopter_physics` (which already links `flight_core`)
rather than `flight_core` alone, because its round-trip checks need Milestone 9's real
`computeStateDerivative()` — see `tests/CMakeLists.txt`'s comment at that target. `firmware/` is
untouched; `idf.py build` is unaffected by this milestone.
