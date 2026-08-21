# Simulator closed-loop stabilization

Milestone 13 wires every piece Milestones 7-12 built into one real closed loop running against
Milestone 9's rigid-body physics simulator: `simulator/sensors/`'s new `SimulatedImu` feeds
noise-corrupted `ImuSample`s to Milestone 8's `AttitudeEstimator`, whose output drives Milestone
11's `AttitudeController` -> Milestone 10's `RateController` -> Milestone 12's `ControlAllocator`,
whose output drives Milestone 9's dynamics for the next timestep. `simulator/sim_loop/`'s
`SimLoop` is the class that ties this together; `tests/sim_loop_test.cpp` demonstrates genuine
convergence (not a single-step sanity check); `simulator/main.cpp` (`bicopter_sim`) is a minimal
text-trace demo of the same loop.

This milestone does not invent new control theory or new physics — it wires up and validates what
Milestones 7-12 already built, and documents two real, structural findings that surfaced while
doing so (below), plus one already-known finding (Milestone 12's pitch-authority limitation) that
this milestone had to work around correctly rather than misreport.

## The simulated IMU (`simulator/sensors/`)

`SimulatedImu` (`simulator/sensors/include/simulated_imu.h`, `src/simulated_imu.cpp`) produces
`ImuSample`s (Milestone 8's estimator input type) from Milestone 9's `RigidBodyState` ground truth
plus the body-frame specific force `SimLoop` computes from the dynamics model (see "Specific-force
derivation" below), on a fixed sample schedule.

**What's modeled:**

- **Gaussian per-axis noise** (`SimulatedImuConfig::gyro_noise_stddev_radps` /
  `accel_noise_stddev_mps2`), sampled independently every reading from a seeded `std::mt19937` /
  `std::normal_distribution` (`SimulatedImuConfig::seed`) — deterministic and reproducible across
  runs and test executions, not time-seeded.
- **Constant per-axis bias** (`gyro_bias_radps` / `accel_bias_mps2`), added on top of noise every
  reading — a fixed zero-offset, the same role as Milestone 3's
  `mpu6050_convert_config_t.accel_offset_mps2`/`gyro_offset_radps` fields, or
  `ComplementaryFilterConfig::gyro_bias_radps`/`accel_bias_mps2` (Milestone 8) — except here it
  models what a *real, uncalibrated* sensor would add, for the estimator's own bias fields to
  (optionally) correct.
- **A fixed sample rate** (`sample_period_s`, default 2 ms / 500 Hz, matching `SensorTask`'s real
  target rate — `firmware/main/task_config.h`, `docs/architecture.md`'s FreeRTOS task table).
  `SimulatedImu::update()` is called every physics tick and returns `std::nullopt` until a full
  sample period has elapsed, mirroring real hardware's fixed output rate rather than a sample
  every physics step.

**What's deliberately simplified** (per the milestone brief's "does not need to be elaborate"
scope):

- No cross-axis noise coupling — each axis's noise is an independent draw.
- No temperature or scale-factor error model.
- No sensor delay/latency beyond the sample-rate quantization itself (a reading at time `t` is
  computed from the state at exactly `t`, not the state from a few sample periods earlier the way
  a real ADC/filter pipeline might introduce).
- No `Barometer` — attitude-only, matching Milestone 8's estimator scope; barometer-based
  altitude/vertical-speed estimation remains deferred (`docs/estimation.md`, `docs/dynamics.md`).

### Specific-force derivation

An accelerometer measures specific force (thrust + drag, excluding gravity's own contribution),
in body frame. `SimLoop` derives it from Milestone 9's `computeStateDerivative()`:

```
specific_force_world = acceleration_mps2 - (0, 0, g)     // g = 9.80665 m/s^2, NED
specific_force_body  = orientation.inverse().rotate(specific_force_world)
```

(`acceleration_mps2 - (0,0,g)` cancels gravity's own contribution to `acceleration_mps2` exactly,
since `F_gravity_world / mass == (0,0,g)` identically — see `docs/dynamics.md`'s "Forces"
section.) This matches `docs/estimation.md`'s existing "gravity is +Z is down, accel reads -Z when
level" convention: a stationary, level, hovering vehicle has `acceleration_mps2 == 0`, giving
`specific_force_body == (0,0,-g)`, exactly what a level, at-rest accelerometer reads.

## Why the estimator is seeded with the true initial attitude

`tests/sim_loop_test.cpp` and `simulator/main.cpp` both call `SimLoop::reset(initial_state,
initial_state.orientation)` — seeding the estimator's starting orientation to match the
*true* disturbed initial attitude, not `Quaternion::Identity()`. This models a real flight
controller's power-on attitude self-leveling from the accelerometer *before* motors spin up to
hover thrust — exactly the stationary, non-thrusting scenario `docs/estimation.md`'s own
`tests/complementary_filter_test.cpp` validates (`test_stationary_convergence`). It is not feeding
the controller ground truth during flight: from that seed onward, the estimator is updated
exclusively through `SimLoop::step()`'s normal `update(ImuSample)` calls, using noisy, rate-limited
sensor data like any other caller.

This choice is necessary, not cosmetic — see the next section for why.

## Why the estimator runs with zero accelerometer correction gain during flight

The single most important finding of this milestone. Seeding the estimator at `Identity()` (the
naive choice) while the true vehicle starts disturbed produces an estimator that never detects the
disturbance at all: with zero initial angular velocity and zero commanded torque (since the
estimate says "level," so the attitude controller commands nothing), the gyro reads ~0 forever
(nothing is rotating) and the accelerometer is *structurally blind* to the true tilt. This is not
noise or a tuning artifact — it is provable:

```
specific_force_body == R(q_true)^-1 * (R(q_true) * F_thrust_body / mass) == F_thrust_body / mass
```

Because this vehicle's thrust is generated along a **body-fixed axis** (approximately body -Z,
Milestone 9's `motorThrustDirectionBody()`), `R(q_true)` and `R(q_true)^-1` cancel *exactly*,
regardless of the vehicle's true orientation, whenever the commanded thrust vector is genuinely
fixed in body-frame terms (as it is at hover trim, symmetric throttle, ~zero tilt). The
accelerometer reads the same thing — `F_thrust_body / mass`, ~`(0,0,-g)` at hover trim — no matter
how the vehicle is actually oriented in the world. A complementary/Mahony filter's accelerometer
correction compares this measurement against `q_estimate^-1 * world_up`; since the measurement is
orientation-independent, this comparison is only ever satisfied (zero correction) when
`q_estimate ~= Identity` — meaning nonzero `Kp` **actively pulls a correct, gyro-tracked estimate
back toward "level," regardless of the true attitude.**

This was confirmed empirically before the fix: with `ComplementaryFilterConfig`'s real default
(`kp = 2.0`) and an estimator seeded at the true initial attitude, the true attitude error
initially converges correctly (gyro-tracked, using real commanded torque) but then **stalls and
reverses** partway through recovery, while the *estimated* error is dragged down to ~0 — i.e. the
controller is fooled into thinking it has already converged. Setting `kp = 0` (pure gyro
integration, `ki = 0` too, since the same corrupted error signal also feeds the online bias
adapter) resolves this completely; see the convergence results below.

This is a hovering-flight-specific **extension** of `docs/estimation.md`'s already-documented "yaw
is unobservable from gravity alone" finding to roll/pitch as well: that finding is about a
rotation *about* the gravity axis leaving the accelerometer reading unchanged; this finding is
about a vehicle whose own thrust *is* the dominant specific-force contributor, which masks
gravity's direction entirely regardless of which axis the vehicle has rotated about. It does not
contradict or invalidate Milestone 8's complementary filter or its tests — those exercise exactly
the regime (`stationary, non-thrusting`) where accelerometer correction is valid and necessary;
this milestone's finding is that the *same* filter needs `Kp` reduced (here, to zero) once the
vehicle is airborne and thrust-bearing. This is scoped to `SimLoop`'s configuration only — the
`ComplementaryFilterConfig` *default* (`kp = 2.0`) is unchanged, since it is correct for the
scenario it was designed and tested against.

**What a real vehicle (Milestone 17) would need instead:** either (a) calibrate attitude from the
accelerometer only while disarmed/at rest (as this milestone's seeding models), relying on gyro
integration alone during flight (viable over short flights with a low-drift MEMS gyro and static
bias calibration, which is what this vehicle's `gyro_bias_radps` config already supports), or (b)
a smarter fusion approach — e.g. an EKF that treats the *commanded* thrust as a known input and
subtracts its expected specific-force contribution before using the residual as a gravity
reference — which is real, scoped future work (`docs/estimation.md`'s EKF-drop-in interface
already anticipates a replacement estimator), not something this milestone's complementary filter
attempts.

## Why the vehicle configuration needs nonzero drag

`docs/dynamics.md`'s "Drag" section already anticipated this: *"a nonzero value is there for
whichever later milestone (13's closed-loop stabilization, most likely) wants some velocity
damping for a more well-behaved simulated vehicle."* This milestone is that case.

A combined roll+yaw disturbance excites real gyroscopic cross-coupling
(`I*omega_dot + omega x (I*omega) = tau`, `docs/dynamics.md`) between axes, because
`inertia_diag_kg_m2` is deliberately asymmetric (`(0.02, 0.03, 0.04)`, matching
`tests/bicopter_dynamics_test.cpp`'s/`tests/control_allocator_test.cpp`'s fixture). This leaks a
small, steady angular rate into **pitch** — an axis the default (zero CoM offset) vehicle has *no*
torque authority over at all (`docs/control_allocation.md`'s Milestone 12 finding). With zero
drag, nothing opposes that leaked pitch rate once it appears, so it persists indefinitely and
pitch angle grows *without bound*, even after roll and yaw both fully settle (confirmed
empirically: pitch rate stabilized at a small nonzero constant while roll/yaw rates decayed to
~0, and pitch angle grew linearly for the whole test duration). Configuring
`linear_drag_coefficient_n_per_mps = 0.2` and `angular_drag_coefficient_nm_per_radps = 0.05` (both
zero in `tests/bicopter_dynamics_test.cpp`'s/`tests/control_allocator_test.cpp`'s fixtures, which
need the undamped case for their exact closed-form checks) gives passive aerodynamic damping that
lets the leaked pitch rate decay to zero once the roll/yaw maneuver settles, bounding pitch at a
small residual offset instead of growing forever.

This is not a workaround for a bug in `ControlAllocator` or `RateController` — it's the same
"structural, not invented" finding `docs/control_allocation.md` already documents (no pitch
authority without a CoM offset), now observed as a *dynamic* consequence (gyroscopic coupling
during a multi-axis maneuver) rather than only a *static* one (an initial pitch disturbance). A
real vehicle in this configuration would have exactly the same property; adding drag doesn't
change that, it just keeps the simulated vehicle's uncontrolled axis from diverging without bound,
which is what lets a bounded-duration test make a bounded-error assertion at all.

## Vehicle configurations demonstrated

Both configurations below add `linear_drag_coefficient_n_per_mps = 0.2f` /
`angular_drag_coefficient_nm_per_radps = 0.05f` to the otherwise-unmodified `baseParams()` fixture
already established by `tests/bicopter_dynamics_test.cpp`/`tests/control_allocator_test.cpp`
(mass 2 kg, diagonal inertia `(0.02, 0.03, 0.04)` kg·m², motor arms at `±0.15` m, thrust
coefficient 20 N, opposite motor spin, `±90°` tilt limits — chosen there to keep tilt saturation
out of the way while testing allocation math, reused here unchanged rather than inventing new
numbers).

- **Default (zero `center_of_mass_offset_m`)** — `tests/sim_loop_test.cpp`'s `baseParams()`,
  `simulator/main.cpp`'s `demoVehicleParams()`. Per `docs/control_allocation.md`, this
  configuration has roll and yaw authority near hover but **no pitch authority at all**. Used for
  `test_roll_disturbance_converges` and `test_roll_yaw_disturbance_converges`.
- **Nonzero CoM offset** — `tests/sim_loop_test.cpp`'s `comOffsetParams()` (`baseParams()` plus
  `center_of_mass_offset_m = (0, 0, 0.05)`). Per `docs/control_allocation.md`, this gives the
  vehicle real pitch authority. Used for `test_pitch_disturbance_converges_with_com_offset` — an
  **additional** case, not a replacement for the two default-config cases above.

## The closed loop (`simulator/sim_loop/`)

`SimLoop` (`simulator/sim_loop/include/sim_loop.h`, `src/sim_loop.cpp`) steps the loop the
milestone brief describes: get true state -> derive specific force -> sample the IMU if due ->
feed the estimator -> (if a control cycle is due) run
`AttitudeController -> RateController -> ControlAllocator` -> apply the resulting command to
Milestone 9's dynamics for the next `physics_dt_s` -> advance time and repeat.

**Cadence.** `SimLoopConfig::physics_dt_s` (default 2 ms / 500 Hz) is both the physics integration
step *and* the simulated IMU's sample period — this milestone keeps these equal (one "tick" is
both a physics step and a candidate IMU sample) rather than sub-stepping physics between samples,
judged an acceptable simplification given no controller-timing-sensitivity issue arose in testing.
`SimLoopConfig::control_period_s` (default 4 ms / 250 Hz, matching `FlightControlTask`'s real
target rate) is rounded to the nearest whole number of physics/IMU ticks (2, by default) —
`SimLoop` runs the attitude/rate/allocation cascade only every `control_decimation_`-th IMU
sample, holding the previous actuator command via zero-order hold in between, mirroring
`SensorTask`'s 500 Hz / `FlightControlTask`'s 250 Hz split (`docs/architecture.md`'s FreeRTOS task
table) rather than assuming estimation and control run at the same rate.

**Desired thrust.** `SimLoopConfig::desired_thrust_n` (default: `<= 0` means "hover," computed as
`vehicle_params.mass_kg * 9.80665` at `reset()`) is the constant weight-supporting thrust demand
fed to `ControlAllocator` every control cycle — this milestone has no altitude/throttle source
(a later milestone's job), so hover thrust is the only demand exercised, per the milestone brief's
item (g).

## Convergence criteria

`tests/sim_loop_test.cpp`'s `runConvergenceCase()` runs the closed loop from a disturbed initial
attitude toward level (`Quaternion::Identity()`) for `duration_s` of simulated time, logging the
**true** (ground-truth, not estimated) attitude error — the geodesic angle
(`quaternionAngleRad()`, `2*acos(|q_error.w|)`, radians in `[0, pi]`) between
`RigidBodyState::orientation` and the desired attitude — at every control cycle. It asserts two
things, not one:

1. The error drops to at or below `tolerance_rad` by `settle_by_s`.
2. The error **stays** at or below `tolerance_rad` at *every* logged cycle from `settle_by_s`
   through the end of the run — i.e. a real converge-and-hold, not a single touch followed by
   drift or divergence (the failure mode a naive "did it ever get close once" check would miss).

Ground truth, not the estimate, is used for both checks — this is what makes the test a real
control-loop convergence check rather than a self-referential "does the estimator agree with
itself" check. `SimStepLog::estimated_attitude_error_rad` is logged alongside for observability
(and printed by `simulator/main.cpp`'s demo) but is not part of the pass/fail criteria.

| Test | Vehicle | Disturbance | `duration_s` | `settle_by_s` | `tolerance_rad` | Result (max tail error) |
|---|---|---|---|---|---|---|
| `test_roll_disturbance_converges` | default | 20° roll | 4.0 | 2.0 | 5° | 0.60° |
| `test_roll_yaw_disturbance_converges` | default | 15° roll + 20° yaw | 4.0 | 2.5 | 6° | 4.64° |
| `test_pitch_disturbance_converges_with_com_offset` | CoM offset | 15° pitch | 4.0 | 2.5 | 6° | 0.26° |

The roll+yaw case's residual (~4.6°, vs. roll-only's ~0.6°) is exactly the bounded pitch coupling
described above — a real, physically-caused residual from the gyroscopic leakage into an
uncontrolled axis, not test slop; its tolerance is set correspondingly looser and documented as
such here rather than silently tightened by retuning something else to hide it.

Sensor noise for all three tests (`testImuConfig()`): `gyro_noise_stddev_radps = 0.005` rad/s,
`accel_noise_stddev_mps2 = 0.02` m/s² per axis, zero bias, one fixed seed per test (1/2/3) for
reproducibility. Zero bias is deliberate: a nonzero gyro bias would leave a permanent yaw
steady-state offset (yaw has no accelerometer-based correction at all — `docs/estimation.md` — and
this milestone's `kp = 0` means roll/pitch get none either during flight), which is a real,
meaningful thing to characterize but a different question from whether the control cascade itself
converges given an accurate-enough attitude estimate.

## Gain tuning: the placeholder gains were not retuned

`docs/control.md`'s `RateController`/`AttitudeController` default gains
(`detail::DefaultRateAxisGains()`: `kp=0.08, ki=0.02, kd=0.004`, torque limits `±0.5` N·m;
`detail::DefaultAttitudeKp()`: `(4.0, 4.0, 4.0)`; `detail::DefaultRateLimitRadps()`:
`(6.0, 6.0, 6.0)`) were explicitly labeled placeholders pending this milestone's closed-loop
simulator. **They were not changed.** Every convergence issue found during this milestone's work
(documented above) was a sensor-fusion/observability configuration issue — the estimator being
seeded wrong, then fighting itself via a corrupting accelerometer correction, then (once fixed)
an undamped uncontrolled axis accumulating gyroscopic leakage — not a control-gain issue. Once
those were fixed, the existing placeholder `Pid`/`RateController`/`AttitudeController` gains
converged all three test cases with no oscillation, no overshoot beyond the small residuals noted
above, and no saturation-driven instability. `flight_core/control/`'s defaults are therefore left
as documented placeholders, now with an honest additional data point: they behave well against a
real closed-loop simulator, not just in isolated unit tests, but they remain simulator-validated,
not hardware-validated (Milestone 17 is still the bar for that).

## Build

`simulator/sensors/CMakeLists.txt` builds `bicopter_sim_sensors` (guarded `add_subdirectory` of
`../physics`, same pattern as `simulator/physics/CMakeLists.txt` — see AGENTS.md's "CMake: guard a
shared subdirectory" note). `simulator/sim_loop/CMakeLists.txt` builds `bicopter_sim_loop`,
guarding both `bicopter_physics` and `bicopter_sim_sensors` the same way. `simulator/CMakeLists.txt`
now builds both as real subdirectories and builds `main.cpp` as the `bicopter_sim` executable
(previously commented out — Milestone 9 judged a smoke-test executable unnecessary given automated
test coverage; this milestone's brief explicitly asks for a lightweight trace, so it's built now).
`tests/CMakeLists.txt` adds `simulator/sensors` and `simulator/sim_loop` as guarded subdirectories
and a new `sim_loop_test` target linking `bicopter_sim_loop`.

```sh
cmake -S simulator -B simulator/build
cmake --build simulator/build
./simulator/build/bicopter_sim        # text-trace demo

cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

`firmware/` is untouched; `idf.py build` is unaffected by this milestone (simulator-only, per the
milestone brief).
