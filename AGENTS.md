# AGENTS.md

Structural and convention decisions made in Milestone 1 that later milestones must stay
consistent with. This file records decisions, not derivations — see the linked docs for the
reasoning.

## Directory layout

```
firmware/     ESP-IDF project (ESP32, C/C++, native peripheral APIs — not Arduino).
              main/, components/{sensors,actuators,radio,estimation,control,safety,telemetry}/
flight_core/  Platform-independent C++. estimation/, control/, dynamics/, math/, vehicle/,
              safety/. Depends ONLY on the hardware-abstraction interfaces — never on ESP-IDF
              or any desktop-only library. This is the code that flies the vehicle AND the code
              the simulator runs; do not fork it or reimplement it per-target.
simulator/    Desktop C++ simulator. physics/, sensors/, visualization/, main.cpp. Links
              flight_core unmodified against simulated hardware-abstraction implementations.
tests/        Tests, primarily against flight_core.
docs/         architecture.md, hardware.md, control.md, estimation.md.
```

Empty scaffold directories currently contain a `.gitkeep`; remove it in the same commit that
adds the first real file there.

Full rationale: [docs/architecture.md](docs/architecture.md).

## Body-frame / units convention — do not change without updating every doc

- SI units throughout `flight_core`'s internal API (meters, seconds, radians, kg, N, N·m, Pa,
  Tesla). Sensor-native or protocol-native units (degrees, mG, PWM microseconds, raw ADC counts)
  convert at the hardware-abstraction boundary, never inside `flight_core`.
- Body frame: right-handed, **X = forward, Y = right, Z = down**. World frame: NED. Roll about
  X, pitch about Y, yaw about Z.
- This convention is stated in [README.md](README.md) and referenced from every doc that touches
  attitude/control math. If it ever needs to change, update it in all of them in the same
  change, not just the file you're editing.

## Hardware abstraction layer (HAL)

`flight_core` never calls ESP-IDF APIs or touches a specific sensor/actuator part. It only sees
small poll-style interfaces — `Imu`, `Barometer`, `MotorOutput`, `ServoOutput`, `Radio`, and (as of
Milestone 16) a battery monitor: `firmware/components/power/`'s `battery_read()`
(handle-plus-functions, the same shape as `mpu6050_read()`/`bmp581_read()` rather than the
ops-vtable shape `Radio`/`MotorOutput` use), returning a `battery_reading_t` with voltage, percent,
optional current, and LOW/CRITICAL threshold flags — no specific voltage-divider ratio or cell
count assumed (see docs/hardware.md). `firmware/`
implements each against real ESP32 peripherals; `simulator/` implements each against a physics
model. Both link the same `flight_core` sources unmodified.

Rule of thumb for "does this belong in `flight_core` or behind the HAL": if it changes when you
swap a physical part (different IMU, different ESC protocol, different servo), it's a `firmware/`
config or a HAL implementation detail, not `flight_core` logic. Full interface sketches and the
reasoning: [docs/architecture.md](docs/architecture.md#hardware-abstraction-layer). Open
hardware questions and where each is meant to become configurable:
[docs/hardware.md](docs/hardware.md).

Until `flight_core` exists, each `firmware/components/*` driver defines its own local plain-C
struct matching its interface's documented shape (e.g. `firmware/components/sensors/include/
imu.h`'s `imu_reading_t`, added in Milestone 3) instead of waiting on a not-yet-real shared
header. These are deliberately free of ESP-IDF types so they double as the boundary pure/testable
logic is written against (see below). When `flight_core` adds the authoritative HAL interface
headers, reconcile each driver's local copy against them in that same change rather than leaving
two shapes to drift apart.

## Driver testing convention

Firmware-side I/O (I2C/SPI transactions, GPIO/ISR behavior) can't be unit tested without real
silicon or a peripheral-accurate emulator (QEMU's `esp32` machine does not model attached I2C
devices). The convention, established in Milestone 3 for the MPU6050 driver: split each driver
into (a) hardware I/O code that depends on ESP-IDF types and is reviewed against the part's
datasheet but not automated-tested pre-hardware, and (b) pure, ESP-IDF-free logic (register
scaling, calibration, staleness/validity checks) that *is* automated-tested, by compiling it
directly (via a relative source path, no `flight_core` dependency yet) into a standalone host
executable under `tests/`, built with plain CMake/CTest independent of ESP-IDF — see
`tests/CMakeLists.txt` and `tests/mpu6050_convert_test.c`. Milestone 4 applied the same split to
the BMP581 barometer driver (`tests/bmp581_convert_test.c`), and Milestone 5 to the ESC/servo PWM
output drivers (`tests/pwm_esc_convert_test.c`, `tests/servo_convert_test.c`,
`tests/pwm_util_test.c`); apply it to later actuator/sensor drivers the same way.

## What's real vs. stub right now

As of Milestone 7: `firmware/` is a real, buildable ESP-IDF project (see below) whose `main/` runs
six FreeRTOS tasks (SensorTask, EstimatorTask, FlightControlTask, RadioTask, TelemetryTask,
SafetyTask) at documented priorities/periods (`firmware/main/task_config.h`,
[docs/architecture.md](docs/architecture.md#freertos-tasks)), with a queue, a task notification,
an `esp_timer`, a mutex, and the task watchdog wired between them. Task *bodies* are still stub/
no-op logic — no estimation, control-loop math, or actuator-driving logic exists in any task yet;
that is milestones 8-12's job. `SensorTask` is the one exception that reaches real driver code: it
calls the real MPU6050/BMP581 driver interfaces from `firmware/components/sensors/` behind a new
`CONFIG_BICOPTER_SENSORS_ENABLED` Kconfig option (default off, since no board is chosen yet) and
runs gracefully without hardware when it's off. `actuators_init_safe()`
(`firmware/components/actuators/`) is still not called from `main/` — it needs a per-board PWM
pin config that doesn't exist yet (see [docs/hardware.md](docs/hardware.md)'s open ESC/servo pin
items); the natural call site is documented in `firmware/main/main.c` for whichever milestone
finalizes the board config. No radio code exists yet. `flight_core/CMakeLists.txt` builds a real
static library — `flight_core/math/` (vectors, quaternions, small matrices; see
[docs/math.md](docs/math.md), Milestone 7) and, as of Milestone 8, `flight_core/estimation/` (a
Mahony-style nonlinear complementary-filter attitude estimator behind the `AttitudeEstimator`
interface, see [docs/estimation.md](docs/estimation.md)) are real content. Nothing in `firmware/` consumes `flight_core` yet — `EstimatorTask`
(`firmware/main/estimator_task.c`) still only logs/discards each sensor sample; wiring it to
actually call `ComplementaryFilterEstimator` is a noted follow-up (needs `flight_core` wired as
an ESP-IDF component plus a C/C++ boundary adapter converting `imu_reading_t`'s microsecond
timestamps to `ImuSample`'s SI-seconds ones — see docs/estimation.md's "Firmware wiring" section),
not yet done. As of Milestone 9, `flight_core/vehicle/` adds `VehicleParams`/`MotorParams`
(header-only config data, see [docs/dynamics.md](docs/dynamics.md)), and `simulator/physics/` is
real: `bicopter_physics`, a standalone CMake static library (`simulator/physics/CMakeLists.txt`,
same pattern as `flight_core/CMakeLists.txt`) implementing the rigid-body forward-dynamics model
and linking `flight_core` for `Quaternion`/`Vec3`/`VehicleParams`. `simulator/CMakeLists.txt`
configures `physics/` as a real subdirectory (standalone-buildable via
`cmake -S simulator -B simulator/build`); `simulator/sensors/`, `simulator/visualization/`, and
`simulator/main.cpp` (the interactive `bicopter_sim` executable) remain unimplemented — no
simulated `Imu`/`Barometer` HAL implementations exist yet, so nothing yet drives `flight_core`'s
estimator from simulated data. As of Milestone 10, `flight_core/control/` adds `Pid` (a generic
single-axis PID with configurable gains, output saturation, clamping/conditional-integration
anti-windup, and a configurable derivative-on-measurement-vs-error convention — see
[docs/control.md](docs/control.md)) and `RateController` (three independent `Pid` instances, one
per body axis, consuming `AttitudeEstimate::angular_velocity_radps` and a desired rate, producing
a desired body torque). As of Milestone 11, `flight_core/control/` also adds `AttitudeController`
— a stateless proportional quaternion-feedback outer loop (`q_error =
current.inverse() * desired`; see docs/control.md for why that multiplication order, not the
reverse, is the one consistent with `Quaternion::integrate()`'s right-multiply body-rate
convention) converting a desired/current attitude quaternion pair into a desired body rate,
feeding `RateController::update()`'s `desired_radps` argument unchanged. As of Milestone 12,
`flight_core/control/` also adds `ControlAllocator` (`include/control_allocator.h`,
`src/control_allocator.cpp`) — converting a desired total thrust + body torque into motor1/2
throttle and motor1/2 tilt (Milestone 5's `MotorOutput`/`ServoOutput` normalized units), derived
term-by-term (a small-angle linearization around hover, decoupled into a thrust/roll 2x2 solve
followed by a tilt/pitch/yaw 2x2 solve) from Milestone 9's exact forward-dynamics equations, not
invented; see [docs/control_allocation.md](docs/control_allocation.md) for the full derivation,
including the milestone's central finding that this vehicle's geometry has no pitch-torque
authority near hover unless `VehicleParams::center_of_mass_offset_m` has a nonzero vertical
component (a real hardware/geometry finding for Milestone 17, not an allocator limitation).
Getting this derivation to reuse Milestone 9's exact thrust-direction formula (rather than
duplicating it) required moving `motorThrustDirectionBody()` from `simulator/physics/` down into
`flight_core/vehicle/include/motor_geometry.h` — `simulator/physics/`'s `bicopter_physics` target
links `flight_core`, never the reverse, so shared code has to live on the flight_core side of that
edge (the same reasoning that put `VehicleParams` there at Milestone 9); any later milestone
needing to share code between `flight_core/` and something that depends on it should follow the
same pattern rather than duplicating. As of Milestone 13, `simulator/sim_loop/`'s `SimLoop` calls
the full `Pid`/`RateController`/`AttitudeController`/`ControlAllocator` cascade against Milestone
9's simulated dynamics, fed by `simulator/sensors/`'s new `SimulatedImu` (noisy `ImuSample`s
derived from `RigidBodyState` ground truth) rather than perfect state — this is the milestone
where the whole stack first runs closed-loop, demonstrated by `tests/sim_loop_test.cpp`'s genuine
convergence tests (attitude error dropping below a tolerance and staying there, not touching zero
once) and `simulator/main.cpp`'s (`bicopter_sim`) text-trace demo. Two real, structural findings
from this wiring work — accelerometer correction being provably uninformative during hover for a
body-fixed-thrust vehicle, and gyroscopic cross-coupling leaking into the (structurally
unauthoritative, per Milestone 12) pitch axis during multi-axis maneuvers — are fully documented
in [docs/simulation.md](docs/simulation.md), which also confirms Milestones 10-11's placeholder
control gains converge as configured, without retuning. `FlightControlTask`
(`firmware/main/flight_control_task.c`) still only reads/logs `safety_state_t` — wiring this same
stack into firmware for real hardware remains unstarted, a distinct (and larger, hardware-adjacent)
piece of work from closing the loop in simulation. `docs/control.md` covers the
`Pid`/`RateController`/`AttitudeController` design and links to
[docs/control_allocation.md](docs/control_allocation.md) for `ControlAllocator`.
`docs/estimation.md` covers attitude estimation as of Milestone 8 and its Milestone 13 update on
hover observability; barometer-based altitude/vertical-velocity estimation remains deferred to a
later milestone (noted in docs/estimation.md, not silently missing). As of Milestone 14,
`firmware/components/radio/` is real: `include/radio.h` is a protocol-independent `Radio` HAL
interface (`has_command`/`get_command`/`get_health`/`deinit`, the same ops-vtable-plus-ctx shape as
`motor_output_t`) so Milestone 15's RC-receiver implementation is a second backend behind it, not a
rewrite; `esp_now_radio.c/.h` is the first concrete implementation (Wi-Fi/ESP-NOW init/pairing,
gated behind `CONFIG_BICOPTER_RADIO_ENABLED`, default off — no ground-station peer paired yet,
same pattern as `CONFIG_BICOPTER_SENSORS_ENABLED`); `radio_packet.c/.h` holds the pure
wire-format/sequence-staleness/packet-loss logic, ESP-IDF-free by design per this file's driver-
testing convention. ESP-NOW's receive callback runs in the Wi-Fi driver's own task context (not an
ISR, but still foreign context this driver must not block or do real work in) and does only a
bounds-checked, non-blocking queue post; all real parsing/validation/sequence-tracking happens in
`esp_now_radio_process_pending()`, called once per cycle from Milestone 6's existing `RadioTask`
(`firmware/main/radio_task.c`) rather than a new task — see [docs/radio.md](docs/radio.md) for the
full callback/task-context writeup, packet format, and pairing procedure. Radio-loss is exposed
through `radio_health_t.link_alive` (a configurable staleness timeout); Milestone 16 still owns
deciding what to *do* about it. No physical ESP32 pair was available in this environment, so real
over-the-air packet delivery/latency/loss/RSSI remain unverified — `idf.py build` was confirmed to
succeed with `CONFIG_BICOPTER_RADIO_ENABLED` both off and on (a real build against ESP-IDF's
`esp_wifi`/`esp_now` libraries), and every piece of pure packet logic has real host-side tests
(`tests/radio_packet_test.c`). As of Milestone 15, `firmware/components/radio/` also has
`crsf_frame.c/.h` (pure CRC8/DVB-S2, byte-level frame sync, `RC_CHANNELS_PACKED` decode, and
channel-to-command calibration, ESP-IDF-free by design — deliberately does not include `radio.h`
since that header transitively needs `esp_err.h`; it defines its own `crsf_command_t` mirroring
`radio_command_t`'s fields instead, the same reason `radio_packet.h` has its own
`radio_command_packet_t` rather than sharing `radio.h`'s type) and `crsf_radio.c/.h` (the second
concrete `Radio` implementation, using ESP-IDF's UART driver in interrupt/ring-buffer mode via
`uart_driver_install()`, not busy-polling). CRSF was chosen over SBUS specifically because classic
SBUS carries no CRC/checksum field at all (only start/end-byte framing) while CRSF has a real
CRC8 — this milestone's explicit requirement — a genuine protocol-choice finding documented in
[docs/radio.md](docs/radio.md), not a detail to silently work around. Unlike ESP-NOW's receive
callback (code this project wrote and had to keep minimal by hand), the UART interrupt/ring-buffer
handoff here is entirely ESP-IDF's own UART driver; `crsf_radio_process_pending()` (called once per
`RadioTask` cycle, same pattern as ESP-NOW) does all real work in that task's own context, never in
driver-internal code. `firmware/main/radio_task.c` wires CRSF in as a build-time either/or
alternative to ESP-NOW behind `CONFIG_BICOPTER_CRSF_RADIO_ENABLED`, with a `#error` guard (confirmed
to actually fire) if both radio options are enabled at once — only one backs `RadioTask`'s single
`radio_t` at a time until a real board decides otherwise. `radio_is_stale()` is reused unchanged for
staleness; packet-loss is estimated from elapsed time against a configured nominal frame period
since CRSF carries no sequence number (a coarser, documented approximation of ESP-NOW's exact
sequence-gap figure). No `radio.h` changes were needed. No physical CRSF receiver was available in
this environment — `idf.py build` was confirmed to succeed with
`CONFIG_BICOPTER_CRSF_RADIO_ENABLED` off (default) and on, and to fail loudly (not silently pick a
backend) with both radio options on; `tests/crsf_frame_test.c` (140 checks) covers the pure logic.
As of Milestone 16, `firmware/components/safety/` is real: `flight_mode.c/.h` (the full
`BOOT`/`DISARMED`/`ARMED`/`STABILIZE`/`ALTITUDE_HOLD`/`FAILSAFE`/`ERROR` state machine),
`arming.c/.h` (every documented arming precondition, all-or-nothing, plus a blocking-mask for
diagnostics), `failsafe.c/.h` (every required condition except task-watchdog failure, each with a
configurable per-condition response defaulting to a safe motor shutdown/disarm), and
`actuator_command_check.c/.h` (defense-in-depth command validation) — all plain, ESP-IDF-free C,
the same driver-testing-convention split every other component here follows, chosen specifically
so `firmware/main/safety_task.c` (also C) can call it directly without needing the same
not-yet-built C++/ESP-IDF-component bridge that has left `flight_core`'s estimator/controller
stack unwired into firmware since Milestones 8/10-13; `flight_core/safety/` stays an empty
scaffold, same status `flight_core/dynamics/` has held since Milestone 1 — see
[docs/safety.md](docs/safety.md)'s "Why firmware/components/safety/, not flight_core/safety/"
section for the full reasoning, since this is a deliberate exception to "safety/control logic
lives in flight_core" worth understanding before a later milestone touches either directory.
`firmware/main/safety_task.c` replaces Milestone 6's always-disarmed stub with this real logic,
wired into its existing 100Hz loop; `safety_state_t` (`firmware/main/safety_state.h`) is extended
in place to carry a real `flight_mode_t` plus diagnostics rather than just a placeholder `armed`
bool, and a new `radio_state.h`/`.c` (same mutex-protected-struct shape as `safety_state.h`, for
the same torn-read reason) lets `RadioTask` publish its latest decoded command/health for
`SafetyTask` to consume — the first live cross-task consumer of `RadioTask`'s output, though still
a narrower thing than the general RadioTask -> FlightControlTask setpoint plumbing this file's
Milestone 14 entry already notes is unstarted. `firmware/components/power/` is also new: ADC-based
battery voltage (and optional current) monitoring behind `CONFIG_BICOPTER_BATTERY_ENABLED`
(default off, no battery/divider hardware chosen yet), using ESP32's line-fitting ADC calibration
scheme (its only supported one — the newer curve-fitting scheme this target doesn't have). Real
live-firmware-data wiring this milestone: IMU/barometer validity and raw gyro rate (`SafetyTask`
peeks `SensorTask`'s sample queue), radio arm command/link health (via `radio_state.h`), and
battery data when enabled. Honestly not wired: estimator validity and attitude — no estimator runs
in firmware yet, so `estimator_valid` is hardcoded false in `safety_task.c`, never faked true,
meaning arming is structurally blocked in today's firmware regardless of hardware state (the
correct behavior for a genuinely incomplete wiring, not a bug); and the invalid-actuator-command
check, since `FlightControlTask` still calls no real allocator. `idf.py build` succeeds with
`CONFIG_BICOPTER_BATTERY_ENABLED`/`CONFIG_BICOPTER_BATTERY_CURRENT_SENSE_ENABLED`/
`CONFIG_BICOPTER_ALTITUDE_HOLD_ENABLED` each off (default) and on; 185 new passing checks across
`tests/flight_mode_test.c`, `tests/arming_test.c`, `tests/failsafe_test.c`,
`tests/actuator_command_check_test.c`, and `tests/battery_convert_test.c` cover the full
transition table, every arming precondition, every failsafe condition, and the battery conversion
math. See [docs/safety.md](docs/safety.md) for the full write-up, including the complete
transition table and the honest wired-vs-not-yet-wired breakdown. As of Milestone 18 (the last
purely-software milestone — Milestone 17's hardware integration requires physical hardware in hand
and is not dispatched until it exists, see TODO.md),
`firmware/main/Kconfig.projbuild` adds a `BICOPTER_OPERATING_MODE` Kconfig choice
(`SIMULATION`/`HARDWARE_TEST`/
`FLIGHT`) resolved entirely at compile time — chosen over a boot-time-latched value specifically
so no runtime signal (a corrupted NVS read, a miswired strap, a stray radio packet) can ever move
the running vehicle into `HARDWARE_TEST`'s relaxed motor-test path, see docs/bench_test.md's "Why
build-time, not boot-time-latched" section — plus a separate `CONFIG_BICOPTER_BENCH_TEST_MOTORS_
DISABLED` option that makes `firmware/components/actuators/src/pwm_esc_output.c` — the only file
that ever calls ESP-IDF's LEDC PWM API for a motor — `#if`-exclude every one of those calls
entirely, a guarantee verified this milestone by `nm`-inspecting the compiled object (zero `ledc_*`
references under BENCH_TEST, all four otherwise), not just documented. A new `firmware/components/
bench_test/` component (a minimal custom UART command parser, not ESP-IDF's `esp_console`,
specifically so the safety-relevant parsing/confirmation logic stays plain, ESP-IDF-free, and
host-tested per this file's driver-testing convention) backs a new seventh FreeRTOS task,
`BenchTestTask` (`firmware/main/bench_test_task.c`, lowest priority, deliberately not
watchdog-registered — it is an interactive diagnostic tool, not part of the real-time
flight-control path) with four independently-usable commands: continuous sensor/radio-command
streaming (read-only, work regardless of mode or arming state), a direct single-servo angle
command (bypasses the control loop, refused in a `FLIGHT` build), and a single-motor `esc_test`
reachable only in a `HARDWARE_TEST` build, gated behind a literal exact `CONFIRM` token distinct
from the normal arm/throttle path, hard-clamped to a low configured throttle, and auto-returning to
idle after a configured duration regardless of whether a second command ever arrives. This
milestone also finally calls `actuators_init_safe()` from `main()` — the call site AGENTS.md's
Milestone 5/6 entries already documented as the natural one — behind a new
`CONFIG_BICOPTER_ACTUATORS_ENABLED` gate (default off, same "no board chosen yet" pattern every
other optional-hardware option here uses), publishing the result via a new, mutex-free
`firmware/main/actuators_state.h`/`.c` (mutex-free because it is written once at boot, before any
task that reads it starts, and never reassigned after — unlike `safety_state.h`/`radio_state.h`,
which are both written every cycle). See [docs/bench_test.md](docs/bench_test.md) for the full
writeup, including the recommended real-hardware bring-up sequence Milestone 17 is meant to run
against.
See [TODO.md](TODO.md) for the full milestone sequence and current status.

## CMake: guard a shared subdirectory before add_subdirectory-ing it

As of Milestone 9, more than one top-level CMake entry point can pull in `flight_core`:
`tests/CMakeLists.txt` adds it directly (for the math/estimation tests), and
`simulator/physics/CMakeLists.txt` also needs it (for `Quaternion`/`Vec3`/`VehicleParams`) —
including it from both `tests/CMakeLists.txt` (which nests `simulator/physics/` as a
subdirectory) and standalone (`cmake -S simulator -B simulator/build`, which reaches
`flight_core` only through `simulator/physics/CMakeLists.txt`'s own `add_subdirectory`) would
`add_subdirectory` the same physical `flight_core/` directory twice in one configure, which is a
hard CMake error (duplicate target). The fix, in `simulator/physics/CMakeLists.txt`:

```cmake
if(NOT TARGET flight_core)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../flight_core ${CMAKE_CURRENT_BINARY_DIR}/flight_core-build)
endif()
```

Any later milestone that reuses a library from more than one entry point (e.g. Milestone 12's
`flight_core/control/` linked from both `tests/` and `simulator/`) should follow the same
guarded pattern rather than assuming there's only ever one path into the directory tree.

## Firmware toolchain

- ESP-IDF **v5.5.5**, target chip **`esp32`** (pinned in `firmware/sdkconfig.defaults`; no
  specific board/pinout chosen yet — see [docs/hardware.md](docs/hardware.md)).
- Install/build/flash instructions and how boot is verified (QEMU, since no physical board is
  available in this environment): [docs/firmware.md](docs/firmware.md).
- `firmware/sdkconfig` (the full generated config) is gitignored; only `sdkconfig.defaults` is
  committed. A fresh checkout just needs `idf.py build` — target is already pinned.

## Maintaining this file

Keep this file proportionate: only record decisions that constrain *future* work across
milestones (conventions, interface shapes, layout) — not implementation narration, not anything
`git log`, the code itself, or the docs it links to already show authoritatively. When a later
milestone changes something recorded here (e.g. extends a HAL interface, revises a task rate),
update the relevant entry in the same change rather than leaving this file stale.
