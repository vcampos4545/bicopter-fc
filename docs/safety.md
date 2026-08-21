# Safety, failsafes, and battery monitoring (Milestone 16)

`firmware/components/safety/` and `firmware/components/power/` implement this project's flight-mode
state machine, arming preconditions, failsafe detection/response, and battery monitoring. This
document covers the full transition table, every arming precondition, every failsafe condition and
its default response, the battery-monitoring model, and — as honestly as the rest of this project's
docs try to be — exactly what is wired to genuinely live firmware data this milestone versus what is
implemented and tested but not yet reachable from real hardware. See
[architecture.md](architecture.md#freertos-tasks) for `SafetyTask`'s place in the task table and
[AGENTS.md](../AGENTS.md) for the directory-layout/testing conventions this milestone follows.

## Scope

- The flight-mode state machine (`flight_mode.h/.c`): `BOOT`, `DISARMED`, `ARMED`, `STABILIZE`,
  `ALTITUDE_HOLD` (Kconfig-gated off by default), `FAILSAFE`, `ERROR`, with a full, tested
  transition table.
- Arming preconditions (`arming.h/.c`): every documented gate that must hold before `DISARMED ->
  ARMED`, plus a bitmask reporting exactly which precondition(s) are currently blocking arming.
- Failsafe detection and configurable response (`failsafe.h/.c`): every condition this milestone's
  brief requires except task-watchdog failure (not representable this way — see below), each with
  a documented default response.
- Defense-in-depth actuator-command validation (`actuator_command_check.h/.c`).
- Battery voltage (and optional current) monitoring (`firmware/components/power/`): ADC-based,
  configurable divider ratio, an approximate voltage-to-percentage curve, and configurable
  LOW_BATTERY/CRITICAL_BATTERY thresholds.
- Real wiring into `SafetyTask` (`firmware/main/safety_task.c`), driving `safety_state_t`
  (extended in place from Milestone 6's stub) every cycle at its documented 100Hz rate.

**Explicitly out of scope:** wiring `flight_core`'s controller/allocator stack (Milestones 10-13)
into firmware's `FlightControlTask` for real actuator output. That stack lives in
`flight_core`/the simulator; `FlightControlTask` still only reads `safety_state_t` and logs. This
milestone makes the safety/arming/failsafe logic itself correct, tested, and ready to gate real
actuator output whenever that separate, larger piece of firmware wiring lands — it does not
implement that wiring itself.

## Why `firmware/components/safety/`, not `flight_core/safety/`

`TODO.md`'s Milestone 16 entry names both `flight_core/safety/` and `firmware/components/safety/`
as the deliverable. This milestone puts the real logic in the latter and leaves
`flight_core/safety/` an empty scaffold (same status as `flight_core/dynamics/` has held since
Milestone 1 — that logic ended up in `simulator/physics/` + `flight_core/vehicle/` instead).

The reason: `flight_core` is a plain C++17 static library, and — per AGENTS.md's "What's real vs.
stub right now" — it is **not yet wired into firmware as an ESP-IDF component**. `EstimatorTask`
and `FlightControlTask` have both carried this exact open item since Milestones 8 and 10-13
respectively: calling `flight_core` from firmware needs a not-yet-built ESP-IDF-component
registration plus a C/C++ boundary adapter, explicitly deferred every time it came up. This
milestone's brief requires `SafetyTask` to **actually drive the flight-mode state machine today**,
not carry the same "wired in flight_core but not called from firmware yet" caveat a fourth time.
Building that C++/ESP-IDF bridge just for `SafetyTask` — while leaving it unbuilt for
`EstimatorTask`/`FlightControlTask` — would be new, out-of-scope infrastructure work for an
inconsistent, partial payoff.

Instead, `firmware/components/safety/` follows this project's established driver-testing
convention (AGENTS.md): plain C, zero ESP-IDF dependency, compiled directly into a standalone host
test binary via `tests/CMakeLists.txt` (the same pattern as `mpu6050_convert.c`,
`radio_packet.c`, `crsf_frame.c`). Being plain C lets `firmware/main/safety_task.c` (also C) call
it directly, with no boundary to build — real wiring, today, at zero new infrastructure cost. If a
future milestone builds the `flight_core`-as-ESP-IDF-component bridge for the estimator/controller
stack, that is the moment to decide whether this logic moves into `flight_core/safety/` to be
shared with the simulator too, or stays firmware-side — not a decision to force now.

## The flight-mode state machine

```
BOOT -> DISARMED -> ARMED <-> STABILIZE <-> ALTITUDE_HOLD (gated off by default)
                       |            |              |
                       +---- any of these three, on failsafe_active ----+
                                          |
                                          v
                                     FAILSAFE
                                          |
                          !arm_command (explicit disarm) only
                                          v
                                     DISARMED

DISARMED <-> ERROR: critical_fault asserted / clears (ground-only - see below)
```

`!arm_command` (the pilot's arm switch/command going false) returns to `DISARMED` in one cycle
from **any** state except `BOOT`/`ERROR`/`DISARMED` itself — including directly out of `FAILSAFE`.
This is deliberately the state machine's single fastest, highest-priority path: disarming is
always simple and fast, regardless of what else is going on, per the design brief. The full,
authoritative transition table (every condition, both directions) is below.

### Full transition table

| From | To | Condition |
|---|---|---|
| `BOOT` | `DISARMED` | Always, unconditionally, on the first cycle. No boot self-test exists yet beyond task/mutex/driver init succeeding, which happens before `SafetyTask`'s first cycle runs at all — reaching the state machine at all already means boot succeeded. |
| `DISARMED` | `ERROR` | `critical_fault` asserted. No real detector sets this in today's firmware (see "Reserved, not yet triggered" below) — the transition is implemented and tested, ready for one. |
| `DISARMED` | `ARMED` | `arm_command && arming_preconditions_met` (every precondition in the next section holds). |
| `DISARMED` | `DISARMED` | Otherwise (no arm command, or any precondition still failing). |
| `ERROR` | `DISARMED` | `critical_fault` clears. Never via `arm_command` — motors are never live in `ERROR`, so there is no "fast disarm" out of it. |
| `ERROR` | `ERROR` | `critical_fault` still asserted. |
| `ARMED` | `STABILIZE` | `mode_request == STABILIZE`. |
| `ARMED` | `ALTITUDE_HOLD` | `mode_request == ALTITUDE_HOLD` and the Kconfig gate is on. |
| `ARMED` | `ARMED` | `mode_request == ALTITUDE_HOLD` while the gate is off (documented no-op — see below), or `mode_request == ARMED_IDLE`. |
| `STABILIZE` | `ARMED` | `mode_request == ARMED_IDLE`. |
| `STABILIZE` | `ALTITUDE_HOLD` | `mode_request == ALTITUDE_HOLD` and the gate is on. |
| `ALTITUDE_HOLD` | `STABILIZE` | `mode_request == STABILIZE`. |
| `ALTITUDE_HOLD` | `ARMED` | `mode_request == ARMED_IDLE`. |
| `ARMED` / `STABILIZE` / `ALTITUDE_HOLD` | `FAILSAFE` | `failsafe_active` (any failsafe condition — see below — is currently true). Checked every cycle, ahead of mode-request handling. |
| `FAILSAFE` | `FAILSAFE` | `failsafe_active` still true, **or** it has cleared but `arm_command` is still true (see "FAILSAFE is latching" below). |
| any of `ARMED` / `STABILIZE` / `ALTITUDE_HOLD` / `FAILSAFE` | `DISARMED` | `!arm_command` (fast disarm — highest priority, checked before failsafe/mode-request handling). |

Invalid transitions that are impossible by construction (verified in `tests/flight_mode_test.c`,
not just asserted here):

- `DISARMED` never transitions directly to `FAILSAFE`, for any input combination — a
  failsafe-worthy condition while disarmed is expected to surface through
  `arming_preconditions_met()` being false instead (see "ERROR is ground-only" below).
  `flight_mode_transition()`'s `DISARMED` branch does not even read `failsafe_active`.
- `FAILSAFE` never transitions directly to `ARMED`/`STABILIZE`/`ALTITUDE_HOLD` — only to
  `DISARMED`, and only via `!arm_command`.
- `ERROR` never transitions to anything but `DISARMED`/`ERROR` — never directly back to a flying
  mode.

### FAILSAFE is latching, by design

Once `FAILSAFE` is entered, clearing the condition that caused it does **not** by itself return the
vehicle to flight — `arm_command` staying true the whole time is not treated as re-arming. The only
way out is the fast-disarm path (`arm_command` going false), followed by the normal `DISARMED ->
ARMED` path, which itself re-checks every precondition from scratch. This is a deliberate reading of
the design brief's "must never auto-arm" requirement: resuming motor output after a failsafe without
a fresh, explicit disarm-then-rearm action is functionally auto-re-enabling flight, even if the arm
switch's position never physically changed. `tests/flight_mode_test.c`'s
`test_failsafe_is_latching_until_explicit_disarm` and
`test_disarm_then_rearm_requires_fresh_preconditions` cover this.

### `ERROR` is ground-only, by design

`ERROR` is reachable only from `BOOT`/`DISARMED`, never from a flying state. A condition serious
enough to matter while airborne is represented as a failsafe trigger (routing to `FAILSAFE`, the
one well-tested in-flight response), never as a second, separately-tested "in-flight ERROR" path.
This keeps the state machine to exactly one in-flight fault path, not two — one motor-shutdown
code path to trust, not a choice between two under time pressure. `critical_fault` is therefore
only ever consulted while `BOOT`/`DISARMED`.

**Reserved, not yet triggered:** no detector in today's firmware sets `critical_fault` true — there
is no config-validation-at-boot check (or similar pre-flight-only fault) implemented yet. The input
field, the transition logic, and its tests all exist and are exercised with synthetic inputs
(`tests/flight_mode_test.c`), ready for whichever future check needs it, the same way
`ALTITUDE_HOLD` was real-but-gated-off before this milestone made it reachable.

### `ALTITUDE_HOLD` is gated off by default

`CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED` (default `n`) gates the mode per the design brief's
"experimental, disabled/unreachable by default" requirement. Requesting it while gated off is a
documented no-op — the state machine stays in whatever mode it was already in
(`flight_mode.c`), not a guess at "idle" vs. "STABILIZE." Even with the gate on,
`ALTITUDE_HOLD` has no real control loop behind it: no barometer-based altitude/vertical-velocity
estimator exists yet (`docs/estimation.md` notes this is deferred). Enabling the gate makes the
*mode* reachable and tested; it does not add altitude-hold *control* — that is a distinct, later
piece of work.

### Radio flight-mode request decoding

`radio_command_t.flight_mode` (a raw `uint8_t`, semantics owned by this layer per `radio.h`'s own
doc comment) is decoded in `safety_task.c`: `0` → `ARMED_IDLE`, `1` → `STABILIZE`, `2` →
`ALTITUDE_HOLD`, anything else → `ARMED_IDLE` (fails closed to the least-authority request, not an
error).

## Arming preconditions

`arming_preconditions_met()` (`arming.h/.c`) is a single all-or-nothing gate — every field below
must be true, with no partial-credit path, per the design brief's "must never auto-arm"
requirement:

| Precondition | Source |
|---|---|
| `imu_valid` | Milestone 3's `imu_reading_t.valid` (not stale/invalid). |
| `estimator_valid` | Milestone 8's `AttitudeEstimate::valid`. |
| `radio_link_alive` | Milestone 14/15's `radio_health_t.link_alive`. |
| `radio_arm_command` | `radio_command_t.arm` — the explicit pilot arm request (required in addition to `radio_link_alive`: a live but never-armed link must not arm on its own). |
| `battery_ok` | Battery voltage above the configured LOW_BATTERY threshold. |
| `no_critical_errors` | Caller-supplied rollup of any other critical condition (e.g. a latched invalid-actuator-command state) not otherwise named above. |
| `is_stationary` | The design brief's explicit "check the vehicle is stationary during arming" requirement — gyro angular-rate magnitude below a configurable threshold (`arming_is_stationary()`, squared-magnitude comparison, no `sqrt` needed). |

`arming_blocking_mask()` returns a bitmask (`ARMING_BLOCK_*`) of exactly which precondition(s) are
currently failing, for logging/telemetry — `safety_state_t.arming_blocking_mask` carries this while
`mode == DISARMED`. Every precondition individually blocking arming, and the all-clear case, are
covered in `tests/arming_test.c` (27 checks).

## Failsafe conditions and default responses

`failsafe_evaluate()` (`failsafe.h/.c`) checks every condition in a fixed, documented priority
order and returns the single highest-priority one currently active (`FAILSAFE_REASON_NONE` if
none). The order exists so a condition whose own inputs would be meaningless under an upstream
failure is always checked *after* that failure — e.g. a dead IMU also reads a zeroed "stationary"
gyro, so `IMU_FAILURE` must outrank `EXCESSIVE_ANGULAR_VELOCITY`'s absence, not the other way
around.

| Priority | Condition | Trigger | Default response |
|---|---|---|---|
| 1 (highest) | `IMU_FAILURE` | `!imu_valid` | `MOTOR_SHUTDOWN` |
| 2 | `ESTIMATOR_FAILURE` | `!estimator_valid` | `MOTOR_SHUTDOWN` |
| 3 | `EXCESSIVE_ATTITUDE` | tilt-from-level angle exceeds a configurable max (0 disables) | `MOTOR_SHUTDOWN` |
| 4 | `EXCESSIVE_ANGULAR_VELOCITY` | body angular-rate magnitude exceeds a configurable max (0 disables) | `MOTOR_SHUTDOWN` |
| 5 | `RADIO_LOSS` | `!radio_link_alive` | `MOTOR_SHUTDOWN` |
| 6 | `CRITICAL_BATTERY` | voltage at/below the configured critical threshold | `MOTOR_SHUTDOWN` |
| 7 | `INVALID_ACTUATOR_COMMAND` | a command that would violate configured limits reached this layer (defense in depth — see below) | `MOTOR_SHUTDOWN` |
| 8 | `BAROMETER_FAILURE` | `!baro_valid` | `WARN_ONLY` |
| 9 (lowest) | `LOW_BATTERY` | voltage at/below the configured low threshold (but above critical) | `WARN_ONLY` |

Per the design brief: **the default response for every condition that threatens controllability is
a safe motor shutdown/disarm, never an attempted autonomous landing.** `FAILSAFE_RESPONSE_LANDING`
exists as a documented, explicitly-not-implemented extension point (`failsafe_response_t`) for a
future autonomous-landing behavior — nothing in this milestone gives it real meaning.

Two conditions default to `WARN_ONLY` instead, for reasons this project's own docs already
establish rather than an arbitrary exception:

- **`BAROMETER_FAILURE`**: the barometer is not in the attitude-control loop at all —
  `docs/estimation.md` explicitly defers altitude/vertical-velocity estimation. Forcing a motor
  shutdown over a sensor that isn't stabilizing the vehicle would be strictly less safe than
  continuing to fly on a good IMU/estimator and losing only altitude telemetry.
- **`LOW_BATTERY`**: a pre-critical warning, not yet an immediate controllability threat.
  `CRITICAL_BATTERY` (a lower, distinct threshold) still defaults to `MOTOR_SHUTDOWN` — forcing a
  shutdown at the first `LOW` crossing, potentially from cruising altitude, would itself be the
  unsafe action here, not the safe one.

Every response is per-condition configurable (`failsafe_config_t.response[]`), not hardcoded —
an integrator who wants uniform `MOTOR_SHUTDOWN` behavior for every condition can set it. The
`battery_voltage_v <= 0` sentinel (see `failsafe_inputs_t`) means "no reading available," not "an
empty pack" — it never triggers either battery failsafe (a caller with no battery ADC configured
reports 0, which must never read as CRITICAL_BATTERY).

Every condition firing correctly, and *not* firing on the corresponding safe/normal input, plus
the priority ordering and the response-lookup/default-config behavior, are covered in
`tests/failsafe_test.c` (37 checks).

### Radio loss: deterministic, tested behavior

`RADIO_LOSS` fires from exactly one input, `radio_health_t.link_alive` (Milestone 14/15's
configurable staleness timeout), independent of whatever stale command value
`radio_command_t.arm`/`.flight_mode` last carried — `radio.h` itself documents that
`get_command()` returns "the last-known value, not link liveness." This means link loss always
routes to `FAILSAFE` deterministically the cycle after staleness is detected, regardless of what
the pilot's stick/switch positions were doing at the moment the link dropped.

### Defense-in-depth actuator-command validation

`actuator_command_is_valid()` (`actuator_command_check.h/.c`) checks a fully-allocated command
(Milestone 5's normalized `MotorOutput`/`ServoOutput` units) against configured throttle/tilt
limits, rejecting any non-finite (NaN/Inf) value regardless of configured range. This is
deliberately redundant with Milestone 5's own clamping and Milestone 12's `ControlAllocator`
saturation policy — the point, per this milestone's brief, is to not trust every upstream layer
perfectly. No firmware caller feeds this from a real allocator today (see "What's wired to live
data" below); it is implemented and tested standalone (`tests/actuator_command_check_test.c`, 12
checks), ready to gate real actuator output whenever that wiring lands.

### Task watchdog failure

Per `docs/architecture.md`'s existing finding (Milestone 6): the ESP-IDF task watchdog's trip
handler runs in a context (`esp_task_wdt_isr_user_handler()`, ISR context) from which this
project's own ISR-discipline rule already forbids doing real work like an actuator write. A
watchdog trip is therefore **not reachable as an app-level `failsafe_reason_t`** the way the other
nine conditions are — there is no `FAILSAFE_REASON_WATCHDOG` in `failsafe.h`, deliberately, rather
than a reason value nothing can ever actually report. This milestone's answer, consistent with
that finding: rely on the watchdog's own panic-reboot path (`CONFIG_ESP_TASK_WDT_PANIC=y`, already
configured in Milestone 6) as the real failsafe for a hung task, and the eventual real design —
noted in `docs/architecture.md` and still applicable — is a hardware-level failsafe (an ESC/
receiver signal-loss timeout, or `actuators_init_safe()`'s idle/neutral output simply persisting
because nothing refreshed it) rather than software issuing writes from the WDT's own trip context.

## Battery monitoring

`firmware/components/power/` implements ADC-based battery voltage (and optional current) sensing,
split into `battery_convert.c` (pure math, host-tested) and `battery.c` (ESP-IDF `esp_adc` oneshot
driver I/O, not host-testable — same split as every other sensor driver, per AGENTS.md's
driver-testing convention).

- **Voltage**: `raw_mv` (already-calibrated ADC millivolts, from ESP-IDF's `adc_cali_*` API) times
  a configurable `voltage_divider_ratio` (`Vbat/Vadc`) — no specific divider ratio is assumed
  anywhere, since no battery/divider hardware is chosen yet (`docs/hardware.md`'s open item).
  ESP32 (this project's pinned target) only supports the *line-fitting* ADC calibration scheme, not
  the newer curve-fitting scheme available on later chips — `battery.c` uses
  `adc_cali_create_scheme_line_fitting()` specifically.
- **Percentage**: linear interpolation across a configurable ascending `(voltage_v, percent)`
  curve, clamped at the ends. This is an explicit **approximation** — real LiPo voltage-to-charge
  curves are famously nonlinear and load-dependent (rest voltage sags under load and recovers at
  idle; the curve's knee shifts with cell age/temperature/discharge rate), none of which this model
  captures. `battery_default_single_cell_curve()` provides a common single-cell (3.0V empty–4.2V
  full) rest-voltage approximation, scaled by `CONFIG_BICOPTER_BATTERY_CELL_COUNT` at init time —
  not manufacturer data for any specific cell.
- **LOW_BATTERY / CRITICAL_BATTERY thresholds**: configured as millivolts-per-cell
  (`CONFIG_BICOPTER_BATTERY_LOW_VOLTAGE_PER_CELL_MV` / `..._CRITICAL_VOLTAGE_PER_CELL_MV`,
  defaults 3500mV/3300mV — common conservative LiPo per-cell figures, not derived from any
  specific cell's datasheet), multiplied by cell count to get the pack-level threshold used by both
  `arming.battery_ok` and `failsafe`'s battery checks.
- **Current sensing**: supported, not required. A second ADC channel through the same linear
  amps-per-volt scaling path (`current_sense_scale_a_per_v`, 0 = disabled — the convention every
  other optional field in this project's config structs uses). No current-sense hardware (shunt
  resistor or hall-effect sensor) is chosen yet, same TBD status as the voltage divider itself.
  Voltage monitoring is this milestone's priority per the brief; current sensing is the
  straightforward, generically-supported extra, not a second full conversion pipeline.

`CONFIG_BICOPTER_BATTERY_ENABLED` (default `n`, no battery hardware chosen yet) gates real ADC
init, same pattern as `CONFIG_BICOPTER_SENSORS_ENABLED`. While disabled, `battery_ok` fails
**closed** (reports `false`, blocking arming) — a missing signal is never treated as "ok," the same
rule `imu_valid`/`radio_link_alive` already follow when their own hardware/link is absent.
`tests/battery_convert_test.c` (55 checks) covers ADC-to-voltage/current scaling, curve
interpolation and out-of-range clamping, the default curve's monotonicity/bounds, threshold
comparisons (inclusive at the boundary), and the full `battery_convert_raw()` pipeline.

## What's wired to live data, and what isn't yet

Consistent with this project's practice of reporting verified-vs-deferred honestly (see
`docs/radio.md`, `docs/hardware.md`) rather than overclaiming:

| Input | Status |
|---|---|
| IMU validity + raw gyro rate | **Real.** `safety_task.c` `xQueuePeek()`s `SensorTask`'s sample queue — safe for any number of concurrent peekers, and never disturbs `EstimatorTask`'s own single-consumer read of the same queue. Reflects real hardware when `CONFIG_BICOPTER_SENSORS_ENABLED` is on, or the honest all-invalid stub `SensorTask` publishes when it's off. |
| Barometer validity | **Real**, same queue peek. |
| Radio arm command + link health | **Real.** `radio_state_get()` (`firmware/main/radio_state.h`, new this milestone), published by `RadioTask` every cycle — the first live cross-task consumer of `RadioTask`'s decoded output (AGENTS.md's Milestone 14 note that "RadioTask does not yet deposit setpoints into FlightControlTask" describes a separate, still-unstarted, larger setpoint-plumbing item; this is a narrower, safety-only publish). |
| Battery voltage/percent/thresholds | **Real** when `CONFIG_BICOPTER_BATTERY_ENABLED` is set; fails closed while disabled (see above). |
| Estimator validity + attitude (tilt-from-level) | **Not wired.** No estimator runs in firmware yet — `EstimatorTask` still only logs/discards every sample (AGENTS.md). `estimator_valid` is hardcoded `false` in `safety_task.c`, **never faked true**. Consequence: `arming_preconditions_met()` can never return `true` in today's firmware, for any hardware state — arming is structurally blocked, not silently unchecked. This is the correct behavior for a genuinely incomplete wiring, not a bug. |
| Invalid-actuator-command check | Implemented and host-tested; not exercised in firmware, since `FlightControlTask` calls no real allocator yet (out of this milestone's scope). Passed as `true` (nothing to reject) in `safety_task.c`. |
| Task watchdog failure | Not representable as an app-level condition at all — see above. |

Because `estimator_valid` never becomes real in firmware this milestone, the `FAILSAFE` state
(and the failsafe conditions gated behind a valid estimator, like `EXCESSIVE_ATTITUDE`) are
likewise unreachable from today's firmware — the vehicle can never arm, so it can never enter a
flying state to fail out of. Every piece of logic that *would* drive those transitions is real and
independently tested (`tests/flight_mode_test.c`, `tests/failsafe_test.c`) against synthetic
inputs representing a fully-wired estimator; only the live data source is missing, and that gap is
the same category of follow-up work AGENTS.md already tracks for `EstimatorTask`/
`FlightControlTask`.

## Verified vs. deferred

- `idf.py build` succeeds with `CONFIG_BICOPTER_BATTERY_ENABLED` off (default), on, and on with
  `CONFIG_BICOPTER_BATTERY_CURRENT_SENSE_ENABLED` also on; and with
  `CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED` off (default) and on — a real build against ESP-IDF's
  `esp_adc` library, not review alone.
- No physical battery/voltage-divider or current-sense hardware was available in this environment
  — real ADC transactions, real voltage-divider accuracy, and real current-sense scaling remain
  unverified (same constraint as every sensor/actuator/radio driver in this project so far — see
  `docs/hardware.md`).
- No physical IMU/radio/battery combination exists to exercise a real end-to-end arm/fly/failsafe
  cycle on hardware. Every piece of pure logic (flight-mode transitions, arming preconditions,
  failsafe triggers, battery conversion math) has real, passing host-side tests — 185 checks across
  `tests/flight_mode_test.c` (54), `tests/arming_test.c` (27), `tests/failsafe_test.c` (37),
  `tests/actuator_command_check_test.c` (12), and `tests/battery_convert_test.c` (55).
