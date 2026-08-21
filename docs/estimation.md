# Estimation

Milestone 8 lands `flight_core/estimation/`: an attitude estimator fusing gyroscope and
accelerometer readings into a quaternion attitude + angular-velocity estimate. This document
covers the algorithm, the math, the tuning parameters, why this design over an EKF, and the
interface a future EKF-based estimator would need to implement to be a drop-in replacement.

## Scope: attitude only, this milestone

Per the top-level design brief's "initially focus on attitude stabilization" guidance, this
milestone covers **attitude (orientation + angular velocity) only** — not altitude or vertical
velocity. Barometer-based altitude/vertical-speed estimation (fusing `Barometer` pressure readings
with accelerometer data for a smoother, faster estimate than the barometer alone, per
[architecture.md](architecture.md#hardware-abstraction-layer)) is deliberately deferred to a later
milestone, not silently omitted — `AttitudeEstimator`'s inputs/outputs below have no altitude
field, and no barometer-fusion code exists yet. `flight_core/estimation/` will likely grow a
parallel or additive interface for that later rather than folding it into `AttitudeEstimator`.

## Algorithm: a Mahony-style nonlinear complementary filter

`ComplementaryFilterEstimator` (`flight_core/estimation/include/complementary_filter.h`,
`src/complementary_filter.cpp`) does two things every update:

1. **Gyro integration** propagates the current orientation forward using
   `Quaternion::integrate()` (Milestone 7's exact exponential-map body-rate integrator, see
   [math.md](math.md)) — accurate and drift-free over a single step, but any real gyro has a
   nonzero bias, so this alone accumulates orientation error over time (classic gyro drift).
2. **Accelerometer-based correction** counteracts that drift. Rather than the "classic"
   complementary filter's approach (independently estimate an accel-derived attitude, e.g. via
   Euler angles, then blend the two with a crossover frequency), this implementation follows
   Mahony's nonlinear formulation: compute a small-angle rotation error between the measured and
   expected gravity direction, and feed it back as an angular-rate correction added directly to
   the gyro measurement before integration. This is mathematically an "equivalent explicit
   nonlinear correction step" per the milestone brief, and was chosen over Euler blending
   specifically to avoid gimbal-lock artifacts in the correction term itself (Euler-angle blending
   would inherit `toEulerZYX()`'s pitch = ±90° singularity into the correction path; the
   quaternion/vector formulation below never constructs an Euler angle at all).

### The math

Given the current orientation estimate `q` (body-to-world) and a new sample with body-frame
`gyro` (rad/s) and `accel` (m/s²):

```
corrected_gyro = gyro - gyro_bias_static - gyro_bias_estimate   // calibration + adaptive bias

measured_dir   = normalize(accel)                                // body frame
expected_dir   = q^-1 * world_up                                 // world_up = (0, 0, -1) in NED
                                                                   // (gravity is +Z; a stationary
                                                                   // accelerometer reads the
                                                                   // reaction force, i.e. "up")
error          = cross(measured_dir, expected_dir)                // small-angle rotation, body
                                                                    // frame, that would rotate
                                                                    // "expected" onto "measured"

correction     = Kp * weight * error                              // fed into orientation
gyro_bias_estimate += Ki * weight * error * dt                    // fed into the bias estimate

q(t+dt) = normalize(q(t) * dq(corrected_gyro + correction, dt))   // Quaternion::integrate()
```

`weight` (0..1) is the accel-rejection gate — see below. `angular_velocity` published to callers
is `corrected_gyro` alone (bias-corrected raw gyro), deliberately excluding the `correction` term:
the correction is an orientation-propagation artifact of trusting the accelerometer, not part of
the vehicle's actual angular rate, so it shouldn't leak into what a rate controller reads as
"angular velocity."

`Kp` and `Ki` (`ComplementaryFilterConfig::kp`/`ki`) are the tuning parameters:

- **`Kp`** is the "crossover" knob the milestone brief asks for: it sets how strongly the
  accelerometer pulls orientation back toward level relative to how much the gyro is trusted to
  integrate freely. `Kp = 0` degenerates to pure (drifting) gyro integration; a very large `Kp`
  approaches "just use the accelerometer's instantaneous tilt every step" (noisy, and actively
  wrong under real acceleration — see below). The default (`2.0`) converges a several-degree
  initial attitude error within a couple of seconds at the project's 500 Hz `SensorTask` rate
  (`tests/complementary_filter_test.cpp`'s `test_stationary_convergence`) without visibly
  amplifying reasonable sensor noise (`test_noisy_input_convergence`).
- **`Ki`** (default `0`, i.e. off) optionally adapts an online gyro-bias estimate from the same
  error signal, on top of `gyro_bias_radps`'s static calibration offset — the standard Mahony
  integral term. It's left disabled by default because Milestone 3's calibration hooks
  (`accel_offset_mps2`/`gyro_offset_radps`, see below) are the currently-expected source of bias
  correction; enabling `Ki` is a later tuning decision once real hardware data exists to validate
  it against; it's part of the interface now specifically so that tuning doesn't require an API
  change.

### Why gravity is "+Z is down, accel reads -Z when level"

`world_up = (0, 0, -1)` follows directly from this project's body/world-frame convention
(README.md, [math.md](math.md)): NED world frame means +Z is down, so the gravity vector itself is
`(0, 0, +g)`. An accelerometer measures specific force, not true acceleration — for a stationary
vehicle, specific force is the reaction to gravity, i.e. `-gravity_vector = (0, 0, -g)`, pointing
"up." A physical MPU6050 sitting level, with this project's body Z-axis pointing down, reads
approximately `-1g` on its Z axis at rest — this is exactly the sign convention
`tests/complementary_filter_test.cpp` encodes as `kLevelAccel`.

### Accelerometer gating: the "unreasonable reading" handling

The classic complementary-filter failure mode this milestone's brief calls out explicitly: an
accelerometer reads the vector sum of gravity *and* any real linear acceleration (thrust, a bump,
aggressive maneuvering). Blindly trusting `accel` as "the direction of gravity" whenever the
vehicle is actually accelerating hard corrects the attitude estimate *toward a wrong answer*,
which is worse than not correcting it at all.

`ComplementaryFilterEstimator` handles this by gating the correction weight on how close
`|accel|` is to `1g` (`config.gravity_mps2`), via two configurable bands
(`ComplementaryFilterConfig`):

```
deviation = |  |accel| / gravity_mps2  -  1  |

weight = 1                                                     if deviation <= accel_trust_band_g
weight = 0                                                     if deviation >= accel_reject_band_g
weight = linear taper between the two                          otherwise
```

Defaults: full trust within 10% of 1g, zero trust beyond 30% of 1g, linear in between. At
`weight = 0` the estimator falls back to pure gyro integration for that update — it does not
freeze or reject the sample outright, so orientation still tracks real rotation during a
high-acceleration event, it just stops trusting the accelerometer's *direction* as a gravity
reference until the reading looks plausible again. `weight` is exposed via
`lastAccelWeight()` for tests/telemetry to observe directly.
`tests/complementary_filter_test.cpp`'s `test_high_acceleration_rejection` locks this behavior
down: a sustained 3g reading gets `weight == 0` and leaves orientation untouched (gyro reads zero
throughout), while a contrasting normal ~1g reading (`weight ≈ 1`) does correct a slightly-wrong
initial attitude.

### A fundamental limit, not a bug: yaw is unobservable from gravity alone

The accelerometer measures a single direction (gravity) in body frame — a 2-DOF measurement. It
can correct the two DOFs of tilt (roll/pitch relative to gravity) but is mathematically blind to
rotation *about* the gravity axis (yaw): rotating the vehicle about the current "up" direction
doesn't change what the accelerometer reads at all. This estimator makes no attempt to correct
yaw drift — doing so needs an independent yaw reference (a magnetometer, or a fused GPS-course
input), neither of which exists in this project's sensor set yet (see README.md's target-vehicle
list). `tests/complementary_filter_test.cpp`'s `test_stationary_convergence` seeds a roll/pitch-only
initial error specifically to test what the filter *can* correct; a nonzero initial yaw error
would persist indefinitely by design, which is expected and not a defect to "fix" without adding a
yaw-reference sensor.

**Update, Milestone 13: the same blindness extends to roll/pitch during hover.** Wiring this
estimator into a real closed loop against Milestone 9's dynamics (`simulator/sim_loop/`) surfaced
a broader version of this finding: because this vehicle's thrust is generated along a body-fixed
axis, the accelerometer's reading during hover is `F_thrust_body / mass` **exactly**, independent
of the vehicle's true orientation — not just noisy or approximately gravity-aligned, but
structurally uninformative about *any* attitude axis, not only yaw. Accelerometer correction
remains valid for the stationary, non-thrusting scenario this section's tests already exercise;
it is provably counter-productive once the vehicle is airborne and thrust-bearing. See
[docs/simulation.md](docs/simulation.md#why-the-estimator-runs-with-zero-accelerometer-correction-gain-during-flight)
for the full derivation and how `SimLoop` configures `Kp = 0` (pure gyro integration) during
flight as a result — this estimator's own default `Kp` is unchanged, since it remains correct for
the scenario it was designed and tested against.

## Timestamps and dt

`ImuSample::timestamp_s` is a real timestamp (seconds, monotonic, arbitrary epoch), not an
assumed fixed period — `update()` computes `dt` from consecutive samples' timestamps rather than
hardcoding e.g. `1/500`. This matters because `SensorTask`'s actual cycle time will jitter around
its nominal 2 ms period on real hardware (see [architecture.md](architecture.md#freertos-tasks)),
and because the simulator (Milestone 9+) may step at a different rate than firmware entirely — the
same estimator code must behave correctly either way. Edge cases handled explicitly:

- **First sample**: there's no previous timestamp to diff against, so the first `update()` call
  only records the timestamp and publishes the bias-corrected gyro as `angular_velocity`; no
  orientation integration happens until the second sample.
- **Non-monotonic or duplicate timestamps** (`dt <= 0`): skipped for that cycle rather than
  integrating backwards or dividing by zero.
- **A long gap** (an invalid/stale run of samples followed by a valid one): `dt` is clamped to
  `ComplementaryFilterConfig::max_dt_s` (default 0.1 s) so recovery from a stale sensor can't
  integrate one enormous, inaccurate rotation step in a single call.
- **Invalid samples** (`ImuSample::valid == false`, mirroring `imu_reading_t.valid`): not
  integrated at all; `AttitudeEstimate::valid` is set false for that cycle so callers know the
  estimate is stale, while the last known orientation/angular-velocity is retained rather than
  reset to a default.

## Calibration / bias

`ComplementaryFilterConfig::gyro_bias_radps` / `accel_bias_mps2` are static per-axis offsets
subtracted from each sample before use — the same role as Milestone 3's
`mpu6050_convert_config_t.accel_offset_mps2`/`gyro_offset_radps` fields
(`firmware/components/sensors/include/mpu6050_convert.h`). Whether calibration happens once in the
driver (subtracted before `ImuSample` is even built) or here (subtracted from an
already-uncalibrated `ImuSample`) is a call site decision, not something this estimator enforces —
both offset fields are safe to leave at `Vec3::Zero()` if the driver already calibrates. The
optional `Ki` online bias adaptation described above is a second, independent source of bias
correction on top of these static offsets.

## Interface: how an EKF replaces this later without touching callers

`flight_core/estimation/include/attitude_estimator.h` defines `AttitudeEstimator`, an abstract
interface with exactly three methods:

```cpp
class AttitudeEstimator {
public:
    virtual void reset(const Quaternion& initial_orientation) = 0;
    virtual void update(const ImuSample& sample) = 0;
    virtual AttitudeEstimate estimate() const = 0;
};
```

`ComplementaryFilterEstimator` is the only implementation today. `EstimatorTask` (firmware) and
the simulator's control loop (Milestone 9+) are meant to depend only on this interface — feed
`ImuSample`s via `update()` in timestamp order, read the fused `AttitudeEstimate` via
`estimate()`. A future EKF-based estimator (e.g. an error-state Kalman filter over
orientation/gyro-bias, following the Solà reference `docs/math.md` already aligns this project's
quaternion convention with) would:

- Implement the same three methods, keeping its full state (covariance, bias estimates, ...)
  private to the class — nothing in the interface exposes filter-internal state.
- Keep consuming exactly `ImuSample` as input (or a superset, if a later milestone extends it for
  barometer fusion — see the Scope section above) and producing exactly `AttitudeEstimate` as
  output, so `EstimatorTask`/the simulator's call site needs zero changes beyond constructing the
  new type.
- Be free to add its own constructor-time config struct (mirroring
  `ComplementaryFilterConfig`) for process/measurement noise covariances, gyro-bias random-walk
  parameters, etc. — `AttitudeEstimator` intentionally has no config-shaped methods, exactly so
  each implementation's tuning surface can differ.

## Why a complementary filter first, not an EKF

Per the top-level design brief: start simple, justify anything more complex. An EKF's real
advantage here would be principled sensor fusion under known noise covariances and a
formal, statistically-justified accel-rejection/trust mechanism (via the innovation covariance)
instead of this filter's fixed trust-band heuristic. Against that:

- **No noise-covariance data exists yet.** An EKF's benefit over a well-tuned complementary filter
  is proportional to how well its `Q`/`R` matrices are known; this project has no real MPU6050 on
  hand yet (see AGENTS.md's "what's real vs. stub" and docs/hardware.md) to characterize gyro/accel
  noise from. Guessing covariances now would be tuning theater, not a real improvement.
- **The complementary filter is easier to reason about and debug by hand** — `Kp`/`Ki` are two
  scalars with an intuitive physical meaning ("how much do I trust the accelerometer"), versus a
  6+-state covariance matrix. For a first attitude estimator on a project with no flight history
  yet, debuggability during bring-up matters more than optimality.
- **It's computationally far cheaper** on the ESP32's single-precision FPU — no matrix inversion,
  no covariance propagation — which matters at `EstimatorTask`'s cadence (coupled to
  `SensorTask`'s 500 Hz, see [architecture.md](architecture.md#freertos-tasks)).
- **The interface above makes the EKF path a pure addition, not a rewrite**, later, once real
  sensor noise data and a concrete reason (e.g. needing principled multi-sensor fusion once a
  magnetometer or a second IMU is added) justify the added complexity — exactly the
  "don't jump to a full EKF unless you have a concrete reason" guidance this milestone was scoped
  against.

## Tests

`tests/complementary_filter_test.cpp` (`tests/CMakeLists.txt` links it against the real
`flight_core` static library, same as the Milestone 7 math tests):

- `test_stationary_convergence` — a level, at-rest vehicle seeded with a wrong roll/pitch initial
  attitude converges to within 2° of level and holds it.
- `test_known_constant_rotation` — a constant body-rate gyro input with the accel-correction path
  gated to zero weight (by feeding a magnitude-zero accel reading) integrates to the exact expected
  quaternion, matching `tests/quaternion_test.cpp`'s equivalent `Quaternion::integrate()` property
  test run through the estimator's public interface.
- `test_noisy_input_convergence` — bounded deterministic (reproducible, not flaky) noise on both
  gyro and accel around a stationary/level truth stays within 10° of truth over the second half of
  a 6-second run, and the orientation quaternion never leaves unit norm or produces a non-finite
  value.
- `test_high_acceleration_rejection` — a sustained 3g non-gravity reading gets `lastAccelWeight()
  == 0` and leaves orientation untouched (with zero gyro input), while a contrasting ~1g reading
  gets full weight and does correct a small initial error — proving the gating logic, not just a
  low `Kp`, is what rejects the high-g case.

## Firmware wiring: deferred, not done this milestone

`firmware/main/estimator_task.c` (Milestone 6) still only logs/discards each `sensor_sample_t` —
it does not construct or call `ComplementaryFilterEstimator` yet. This is a deliberate, explicitly
noted deferral rather than an oversight: `flight_core` is a plain CMake static library not yet
wired into ESP-IDF's component build system (see `flight_core/CMakeLists.txt` and
[architecture.md](architecture.md)), `EstimatorTask` and `flight_core` are also on opposite sides
of a C/C++ boundary (`firmware/main/` is plain C; `flight_core` is C++), and `imu_reading_t`'s
microsecond `int64_t` timestamp needs converting to `ImuSample`'s SI-seconds `float` at that
boundary. Doing that wiring properly (an ESP-IDF `idf_component_register` for `flight_core`, a
thin `extern "C"` adapter translating `imu_reading_t`/`barometer_reading_t` into `ImuSample`) is
real, hardware-adjacent integration work distinct from the estimation math itself, and is left as
a small, explicitly-scoped follow-up rather than attempted under this milestone's math/test focus.
`idf.py build` still succeeds unmodified (`firmware/` was not touched this milestone).

## Interim conventions (unchanged from Milestone 1)

- The estimator consumes only IMU-shaped input (`ImuSample`, mirroring the `Imu`
  hardware-abstraction interface from [architecture.md](architecture.md)), never a specific part's
  driver directly, so it runs unmodified against both real hardware (once wired, see above) and
  the simulator's synthetic sensor implementations (Milestone 9+).
- All estimator inputs and outputs use the SI units and body-frame convention from
  [README.md](../README.md).
