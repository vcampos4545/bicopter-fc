# Dynamics

Milestone 9 lands `simulator/physics/`: a rigid-body forward-dynamics model of the bicopter —
given the vehicle's current state and its 4 actuator commands, compute the resulting motion.
This document derives every equation, states every assumption explicitly (per the top-level
design brief's "don't fake physics or control theory" rule), and records what's deliberately
simplified vs. a "real" flight-dynamics model.

Vehicle-specific physical constants (mass, inertia, geometry, coefficients) live in
`flight_core/vehicle/include/vehicle_params.h` (`VehicleParams`/`MotorParams`), not
`simulator/physics/` itself — see [hardware.md](hardware.md)'s "Open / TBD items" table and
"Configuration philosophy" section, both written in Milestone 1: these constants are meant to be
consumed by both this milestone's physics model and `flight_core/control/`'s allocation math
(Milestone 12) from the same struct, so the two never drift apart. Milestone 12 also moved
`motorThrustDirectionBody()` itself down into `flight_core/vehicle/include/motor_geometry.h` for
the same reason (this header still re-exposes it via `#include` for backward compatibility) — see
[control_allocation.md](control_allocation.md).

## Scope: forward dynamics only

Given `(state, VehicleParams, ActuatorCommand)`, compute `state_dot` or the next `state` after a
timestep. Explicitly **not** in scope this milestone:

- **Control allocation** (desired body torque/thrust → motor/servo commands, the inverse
  problem) — Milestone 12.
- **Attitude/rate controllers** — Milestones 10-11.
- **Simulated sensor noise/bias models.** No `Imu`/`Barometer` implementations exist yet;
  `simulator/sensors/` remains unimplemented. A future milestone (13, when the estimator/
  controller stack closes the loop against this physics model) will need to derive simulated
  `ImuSample`s from `RigidBodyState` — that conversion, and whatever noise model it carries, is
  deferred to whichever milestone first needs it, not invented ahead of time here.

## Geometry

Two motors, symmetric about the vehicle's roll axis (body X). Each motor's position is given
relative to a *geometric reference frame* whose origin is the symmetric point on the roll axis
equidistant from both motors: `motor_position_ref = (0, arm_offset_y_m, 0)` — motors sit
directly on the body Y axis, `arm_offset_y_m` negative for the nominal left motor
(`VehicleParams::motor1`) and positive for the nominal right motor (`motor2`), each independently
configurable rather than assumed equal-and-opposite (a real frame could be asymmetric).

The true center of mass need not coincide with that geometric origin (e.g. an off-center
battery), so `VehicleParams::center_of_mass_offset_m` gives its offset, and every motor's moment
arm for `r x F` is:

```
r_motor_from_com = motor_position_ref - center_of_mass_offset_m
```

Gravity acts through the center of mass by definition, so it contributes zero torque regardless
of `center_of_mass_offset_m` — only the thrust/reaction-torque terms below use `r`.

## Tilt-vectoring geometry

Each motor's thrust axis tilts via its own servo. This project assumes **both tilt axes are
parallel to the arm connecting the two motors — a rotation about body +Y** (the same axis for
both motors, since the arms are collinear along Y in a symmetric bicopter frame). This is the
standard bicopter thrust-vectoring layout: tilting a motor fore/aft redirects its thrust within
the vehicle's X-Z (forward/up) plane, which is what gives this configuration pitch and yaw
authority (differential/tilt vectoring) on top of the roll authority differential thrust already
provides — see "What each control axis' authority comes from" below.

At zero tilt, a motor's thrust points along body `(0, 0, -1)` — straight "up," since this
project's body frame is Z-down (README.md). A tilt angle `theta` (`ActuatorCommand`'s
`motor{1,2}_tilt_rad`, clamped to `MotorParams::min_tilt_rad`/`max_tilt_rad`) rotates that
direction by `theta` about body +Y, right-hand rule:

```
thrust_dir_body(theta) = R_y(theta) * (0, 0, -1) = (-sin(theta), 0, -cos(theta))
```

`theta = 0` gives straight up; positive `theta` tilts the thrust vector toward `-X` (aft). The
sign is a documented convention, not a physically-forced choice — `motorThrustDirectionBody()`
(`simulator/physics/include/bicopter_dynamics.h`) is the single place this is computed, and
`tests/bicopter_dynamics_test.cpp`'s `test_thrust_direction` pins it down. Thrust force on the
vehicle is `thrust_n * thrust_dir_body(theta)` — the direction the propeller pushes air is
`-thrust_dir_body`, and by Newton's third law the reactive force on the vehicle (what accelerates
it) is `+thrust_dir_body`, which is why hover works out to thrust along `-Z` opposing gravity
along `+Z` (NED) at zero tilt/attitude.

## Thrust model

`thrust_n = thrust_coefficient_n * throttle^2`, `throttle` the normalized `[0,1]` command
(`MotorOutput::write()`'s convention, Milestone 5). The quadratic relationship is the standard
static-thrust approximation for a fixed-pitch prop driven by a brushless motor without RPM
feedback: thrust scales with the square of prop RPM (from momentum/blade-element theory), and an
uncalibrated ESC's throttle command is roughly proportional to motor RPM in its usable range —
neither of those are exact (real motor/prop/ESC combinations have non-quadratic regions,
especially near 0 and 1), but this is a documented, physically-motivated starting curve, not an
arbitrary one, and `thrust_coefficient_n` is per-motor configurable so it can be replaced with a
measured static-thrust curve fit later without changing the model's shape.

## Reaction torque model

`reaction_torque_nm = torque_coefficient_nm_per_n * thrust_n`, acting about the motor's own
(tilted) thrust axis, direction set by `MotorParams::spin_direction` (`+1`/`-1`, which way the
prop spins). Proportional-to-thrust is the standard simplification (both thrust and reaction
torque scale with roughly the same power of prop RPM, so their ratio is close to constant over a
prop's typical operating range) — `torque_coefficient_nm_per_n` is per-motor configurable for a
measured torque/thrust ratio later.

### Spin direction: not assumed opposite

Unlike a standard quadcopter (which relies on opposite-spin motor pairs to cancel reaction torque
and get yaw authority from differential spin), this bicopter's yaw authority comes from
tilt-vectoring (see "What each control axis' authority comes from" below), not reaction-torque
differencing. `MotorParams::spin_direction` is therefore independently configurable per motor,
not hardcoded opposite:

- **Opposite spin** cancels the two motors' reaction torques exactly under symmetric
  equal-thrust hover (zero net yaw bias at trim) — `tests/bicopter_dynamics_test.cpp`'s
  `test_hover_equilibrium` uses this configuration specifically to get a clean "zero net torque"
  check.
- **Same-direction spin** is an equally valid build choice (e.g. simplifying ESC/prop sourcing to
  one part number) that leaves a small, constant reaction-torque yaw bias at hover trim — real,
  but no different in kind from any other constant disturbance a yaw controller (Milestone 10-11)
  already has to reject.

Neither is imposed by the physics model; both are exercised by different tests below.

## Inertia

`VehicleParams::inertia_diag_kg_m2` is a **diagonal** (principal-axis) tensor — `(Ixx, Iyy, Izz)`
— not a full `Mat3`. This is a deliberate, documented simplification (explicitly permitted by the
milestone brief as an alternative to a full tensor): it assumes the body frame's axes are close
enough to the vehicle's true principal axes that products of inertia (`Ixy`, `Ixz`, `Iyz`) are
negligible. For a bicopter frame with left-right mirror symmetry about the body X-Z plane (true
by construction for the motor/servo layout — see "Geometry" above), `Ixy` and `Iyz` vanish
exactly regardless of mass distribution; only `Ixz` (front-back vs. up-down coupling) could be
nonzero for a real asymmetric fuselage, and is assumed small enough to neglect for this
milestone. This keeps `I*omega`, its cross product with `omega`, and the `omega_dot` solve all
simple per-axis (elementwise) arithmetic — no matrix inverse or general 3x3 linear solve needed.
If a real built vehicle's measured inertia significantly violates this assumption, generalizing
to a full `Mat3` tensor (and a corresponding 3x3 solve for `omega_dot`) is a scoped follow-up, not
a hidden gap — `Mat3` (Milestone 7) already exists to hold one and `Mat3::operator*(Vec3)` already
computes `I*omega`; a solve routine is the only missing piece.

## Drag

A simple **linear** damping model, explicitly *not* real aerodynamics (no dynamic-pressure/
velocity-squared drag, no propwash interaction, no blade-flapping, no ground effect):

```
F_drag_world = -linear_drag_coefficient_n_per_mps * velocity_world
tau_drag_body = -angular_drag_coefficient_nm_per_radps * omega_body
```

Both default to `0` (no drag) unless configured — this milestone's tests exercise the
undamped case throughout (drag would only obscure the exact free-fall/hover/torque checks
below), and a nonzero value is there for whichever later milestone (13's closed-loop
stabilization, most likely) wants some velocity damping for a more well-behaved simulated
vehicle.

## Forces

```
m * v_dot = F_gravity_world + R(q) * (F_thrust1_body + F_thrust2_body) + F_drag_world
```

- `F_gravity_world = (0, 0, mass_kg * g)`, `g = 9.80665 m/s^2` — NED means `+Z` is down, so
  gravity (which pulls the vehicle down) is `+Z`, matching
  [estimation.md](estimation.md#why-gravity-is-z-is-down-accel-reads--z-when-level)'s identical
  convention.
- `R(q)` is the body-to-world rotation (`Quaternion::rotate()`, Milestone 7) — thrust forces are
  computed in body frame (they depend on the vehicle's own tilt-servo geometry) and rotated into
  world frame to sum with gravity/drag, which are naturally world-frame quantities.

## Torques

```
I*omega_dot + omega x (I*omega) = tau_thrust1 + tau_thrust2 + tau_drag_body
tau_thrust_i = r_i x F_thrust_i_body + spin_direction_i * torque_coefficient_i * F_thrust_i_body_direction * thrust_i
```

solved for `omega_dot` (all quantities in body frame, `I` diagonal per "Inertia" above):

```
omega_dot = I^-1 * (tau - omega x (I*omega))   [I^-1 is a per-axis divide for diagonal I]
```

## What each control axis' authority comes from

Not implemented this milestone (that's control allocation, Milestone 12) but worth stating since
it's *why* the geometry above is shaped this way, and what the torque tests below are checking:

- **Roll** (about body X): differential thrust between the two Y-offset motors —
  `test_asymmetric_thrust_induces_roll_torque` below.
- **Pitch** (about body Y): common-mode tilt of both motors (both thrust vectors gain the same
  `-X`/`+X` component) — **but only if `VehicleParams::center_of_mass_offset_m` has a nonzero Z
  component.** [control_allocation.md](control_allocation.md)'s Milestone 12 derivation shows this
  precisely: with both motors positioned purely along body Y (this section's geometry) and the
  default zero CoM offset, `r x F` and the reaction-torque term both have *exactly* zero
  Y-component for any thrust/tilt command — common-mode tilt produces a net body-X force
  (translation) but zero net pitching moment. Pitch-torque authority near hover is a real
  capability of this topology, but only once the vehicle has a genuine vertical offset between the
  motor-mounting plane and its true center of mass; see control_allocation.md for the full
  derivation and why this is a hardware/geometry finding, not a control-allocation limitation.
- **Yaw** (about body Z): differential tilt between the two motors, or (if configured) reaction
  torque — `test_tilt_induces_yaw_and_roll_torque` and `test_reaction_torque_yaw` below both
  demonstrate a yaw-torque contribution, from two different physical mechanisms. Milestone 12's
  allocator commits `thrust_1`/`thrust_2` to the roll/total-thrust solve first, so in practice its
  controllable yaw lever is differential tilt; reaction torque shows up as a fixed bias it
  corrects for rather than a separately-controllable input — see
  [control_allocation.md](control_allocation.md).

## Integration scheme

**Semi-implicit (symplectic) Euler**: each step computes the acceleration/angular-acceleration
from the *current* state, updates velocity/angular velocity first, then uses the *updated*
velocity/angular velocity to advance position/orientation:

```
v(t+dt)     = v(t) + a(t)*dt
p(t+dt)     = p(t) + v(t+dt)*dt          // uses the NEW v, not v(t)
omega(t+dt) = omega(t) + alpha(t)*dt
q(t+dt)     = q(t).integrate(omega(t+dt), dt)   // Quaternion::integrate(), Milestone 7
```

Chosen over plain forward Euler (which uses `v(t)` for the position update too) because
semi-implicit Euler is unconditionally stable for simple oscillatory/damped systems where plain
forward Euler can diverge, at zero extra cost per step — a good default for a simulator that
needs to stay numerically well-behaved across a wide range of configured vehicle parameters
without per-config step-size tuning. It is still only first-order accurate (local error `O(dt^2)`,
global error `O(dt)`), unlike RK4; RK4 (or an adaptive-step method) is a reasonable future upgrade
if a later milestone's control-loop testing needs tighter numerical accuracy at larger step
sizes, but isn't justified yet with no controller in the loop to demand it.

Orientation integration reuses `Quaternion::integrate()` (the exact exponential-map body-rate
integrator from Milestone 7, see [math.md](math.md)) rather than re-deriving quaternion
kinematics here — exactly the point of building the math library first.

`tests/bicopter_dynamics_test.cpp`'s `test_free_fall` locks in the specific numerical behavior
this scheme produces: velocity matches the analytic `v=g*t` *exactly* for constant acceleration
(semi-implicit Euler has zero error for the velocity update under constant acceleration), while
position carries the scheme's documented small `O(dt)` bias above the analytic `0.5*g*t^2`.

## Tests

`tests/bicopter_dynamics_test.cpp` (`tests/CMakeLists.txt` links it against `bicopter_physics`,
which itself links `flight_core` — see `simulator/physics/CMakeLists.txt`):

- `test_thrust_direction` — `motorThrustDirectionBody()` returns a unit vector at every tested
  tilt angle, matches the documented `(0,0,-1)`/`(-1,0,0)` values at `0`/`90 deg`, and never
  produces a body-Y component (tilt is about +Y).
- `test_free_fall` — zero thrust, zero drag: velocity matches `g*t` exactly, position matches
  `0.5*g*t^2` within the integration scheme's documented `O(dt)` bias, and orientation/angular
  velocity stay untouched (no torque).
- `test_hover_equilibrium` — symmetric equal thrust (each motor at half the vehicle's weight),
  zero tilt, opposite motor spin: net acceleration and net torque both ~0.
- `test_asymmetric_thrust_induces_roll_torque` — unequal thrust, reaction torque disabled,
  checked against the hand-derived closed form `tau_x = arm_offset_y_m_span * (t1 - t2)`.
- `test_tilt_induces_yaw_and_roll_torque` — a single tilted, offset motor, reaction torque
  disabled, checked against the hand-derived `r x F` closed form under a nonzero tilt angle.
- `test_reaction_torque_yaw` — a motor at the center of mass (so `r x F` vanishes) with reaction
  torque enabled: the resulting angular acceleration is pure yaw, matching
  `-spin_direction * torque_coefficient * thrust`.
- `test_torque_free_steady_spin_about_principal_axis` — zero external torque, initial angular
  velocity purely along one axis of the diagonal inertia tensor: `omega x (I*omega)` vanishes
  exactly, so angular acceleration is exactly zero (a principal-axis spin is a steady state).
- `test_torque_free_angular_momentum_magnitude_conserved` — a non-principal-axis initial spin,
  integrated for many torque-free steps: `|I .* omega|` (body-frame angular momentum magnitude,
  exactly conserved by the continuous-time Euler's-equations solution) stays within a small
  tolerance of its initial value across the whole trajectory, a sanity check on the full
  dynamics+integration pipeline together, not just a single derivative evaluation.

## Build

`simulator/physics/CMakeLists.txt` builds `bicopter_physics` as a standalone plain-CMake static
library (no ESP-IDF dependency), the same pattern `flight_core/CMakeLists.txt` already
established — see `simulator/physics/CMakeLists.txt`'s comments for why it guards its own
`add_subdirectory(flight_core)` with `if(NOT TARGET flight_core)` (so it configures correctly
whether it's pulled in from `tests/CMakeLists.txt` or the top-level `simulator/CMakeLists.txt`).
`simulator/CMakeLists.txt` now configures `physics/` as a real subdirectory; `simulator/main.cpp`
and the `bicopter_sim` executable remain unbuilt (commented out) — this milestone's brief judged
a smoke-test executable unnecessary given the automated test coverage above, leaving that entry
point for whichever future milestone (13, or earlier if a smoke test becomes useful) needs one.
`firmware/` is untouched; `idf.py build` is unaffected by this milestone.
