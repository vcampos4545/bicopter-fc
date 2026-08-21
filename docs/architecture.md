# Architecture

This document describes the shape of the system as designed in Milestone 1. Nothing described
here is implemented yet except the directory structure itself — see [TODO.md](../TODO.md) for
what lands when. The point of writing this now is to fix the interfaces and task boundaries
before code exists, so later milestones build against a stable shape instead of discovering it
piecemeal.

## Pipeline

The system is a pipeline from physical hardware up through control back down to physical
actuators, with simulation as an alternate bottom layer and safety cutting across all of it:

```
 Hardware              RTOS                 Estimation      Control            Simulation
 (or Simulator)         (FreeRTOS tasks)      (flight_core)   (flight_core)      (simulator/)

 IMU, baro,       -->   SensorTask       -->  EstimatorTask -->  FlightControlTask
 battery ADC                                  (attitude,         (attitude ctrl,
                                                altitude,          rate ctrl,
                                                velocity           allocation)
                                                estimate)               |
                                                                        v
 ESCs, servos     <-----------------------------------------------  MotorOutput /
                                                                     ServoOutput

 Radio (ESP-NOW/  <-->  RadioTask        -->  setpoints into FlightControlTask
 RC receiver)                             <-- telemetry from EstimatorTask/FlightControlTask

                        TelemetryTask    -->  packages state for RadioTask / logging

                        SafetyTask       -->  monitors all of the above, can force disarm /
                                               failsafe outputs regardless of FlightControlTask
```

`firmware/` provides the RTOS layer and real hardware drivers behind the HAL interfaces.
`simulator/` provides a physics model behind the same HAL interfaces, and a thin loop in place
of FreeRTOS tasks (see [Simulator execution model](#simulator-execution-model)). `flight_core/`
provides estimation, control, and safety logic and is identical in both.

## Data flow

The control-relevant data flow, one control cycle:

```
sensors (Imu, Barometer, battery monitor)
  -> raw sample acquisition (SensorTask, or simulator sensor step)
  -> state estimation (EstimatorTask: attitude, angular rate, altitude, vertical speed)
  -> flight mode logic (which setpoint source is authoritative: radio stick input,
     failsafe-commanded setpoint, or a scripted/autonomous setpoint later)
  -> attitude controller (attitude error -> rate setpoint)
  -> rate controller (rate error -> body torque + thrust demand)
  -> control allocation (body torque + thrust demand -> per-motor throttle,
     per-servo tilt angle, respecting actuator limits)
  -> actuators (MotorOutput x2, ServoOutput x2)
```

Radio and telemetry are a side channel into and out of this loop, not part of the critical
path: `RadioTask` deposits the latest setpoint for `FlightControlTask` to read and publishes the
latest state for downlink; a stale or lost radio link is a `SafetyTask` concern (failsafe),
not something `FlightControlTask` blocks on.

## Hardware abstraction layer

`flight_core` depends only on a small set of abstract interfaces — plain C++ types with no
ESP-IDF or OS dependency. `firmware/` and `simulator/` each provide one implementation of each
interface; `flight_core` code is unaware of which implementation it's linked against. This is
what makes "the simulator runs the real flight code" true rather than aspirational.

Working interface shape (refined as later milestones implement them):

- **`Imu`** — `read() -> { accel: Vec3 (m/s^2, body frame), gyro: Vec3 (rad/s, body frame),
  timestamp }`. Multiple IMUs are multiple `Imu` instances; fusion across them is an
  `EstimatorTask` concern, not something the interface itself does.
- **`Barometer`** — `read() -> { pressure: float (Pa), temperature: float (degC), altitude: float
  (m, standard barometric formula relative to a configurable sea-level reference pressure),
  timestamp, valid }`. Revised in Milestone 4 from this document's original sketch (which put
  altitude derivation in `flight_core` and used Kelvin): the driver now derives a simple,
  single-sensor altitude itself (with configurable pressure smoothing) so it's usable standalone
  without a working estimator, and uses Celsius directly. Neither change couples anything to a
  specific part's quirks — the barometric-formula math and filtering are pure, part-agnostic logic
  (`firmware/components/sensors/bmp581_convert.c`). The future state estimator (Milestone 8) is
  still free to fuse `pressure` (or `altitude`) with IMU data for a better-behaved fused estimate;
  see [docs/hardware.md](hardware.md#bmp581-milestone-4) for the full reasoning.
- **`MotorOutput`** — `write(throttle: float in [0, 1])` for one motor/ESC channel. Protocol
  (PWM, DShot, ...) and calibration are hidden behind the implementation.
- **`ServoOutput`** — `write(angle: float, radians, servo-frame)` for one tilt servo. Range
  limits and center-trim are hidden behind the implementation.
- **`Radio`** — `send(telemetry packet)` / `receive() -> setpoint packet, or none`. Both
  ESP-NOW and a future RC-receiver implementation produce the same setpoint representation so
  `FlightControlTask` doesn't know or care which transport is in use.
- **Battery monitor** (voltage/current) — same shape as `Barometer`: a small polled-read
  interface, exact type TBD when the ESC/servo HAL milestone lands, since current sense is often
  wired through the same ADC/telemetry path as the ESCs.

Each interface is intentionally minimal (poll-style `read`/`write`, no callbacks) so that the
simulator's implementation can be a direct function of the physics state at the current
simulated time, with no threading or timing subtleties beyond what `simulator/`'s own loop
imposes.

### Simulator execution model

The simulator does not run FreeRTOS. It steps the physics model at a fixed timestep and, on
each step, presents that instant's state through the `Imu`/`Barometer`/battery-monitor
interfaces, calls `flight_core` exactly as `firmware/`'s tasks would (estimator step, then
control step, at whatever relative rates milestone 13 settles on), and applies the resulting
`MotorOutput`/`ServoOutput` commands back into the physics model for the next step. This keeps
the *interfaces* identical between targets while accepting that the *scheduling* is necessarily
different (real concurrency vs. a deterministic single-threaded loop) — task timing behavior
itself is only validated in QEMU as of milestone 6 (see below); real-hardware scheduling
behavior (actual I2C/PWM transaction time, real interrupt latency) remains unverified until
milestone 17.

## FreeRTOS tasks

Implemented in milestone 6 (`firmware/main/`) exactly as this document sketched in milestone 1,
with concrete numbers picked from the ranges below and task/queue/notification/timer/mutex
plumbing wired up. Task *bodies* remain stub/no-op logic — no estimation, control, actuator-
driving, or real radio/safety logic exists yet (see the "What's real vs. what's still a stub"
column and AGENTS.md's "What's real vs. stub right now"). Every number below is defined once in
`firmware/main/task_config.h` and referenced from there — this table should not drift from that
file; if a later milestone changes one, update both in the same change.

| Task | Priority | Period / rate | What it does now (milestone 6) | What it will do |
|---|---|---|---|---|
| `SafetyTask` | 10 (highest) | 10 ms (100 Hz) | Re-affirms the disarmed state every cycle via the mutex-protected `safety_state_t` and pings `FlightControlTask` via task notification (see below) - no real failsafe condition exists to evaluate yet. | Milestone 16: radio-loss, sensor-fault, and battery-cutoff failsafes; forces disarm regardless of `FlightControlTask`. |
| `SensorTask` | 9 | 2 ms (500 Hz) | Calls the real `mpu6050_read()`/`bmp581_read()` driver interfaces when `CONFIG_BICOPTER_SENSORS_ENABLED` is set (see "Sensor hardware exercised" below); publishes each cycle's `sensor_sample_t` onto a length-1 overwrite queue. | Unchanged in spirit; the queue's consumer (`EstimatorTask`) grows real logic. |
| `EstimatorTask` | 8 | Queue-driven, no independent period (blocks on the sensor queue, ≤10 ms wait to keep feeding the watchdog if `SensorTask` stalls) | Receives each sample and logs/discards it - no estimation math exists yet. | Milestone 8: attitude/altitude estimator built on the `flight_core/math/` library ([docs/math.md](math.md)) landed in milestone 7. |
| `FlightControlTask` | 7 | 4 ms (250 Hz) | Reads `safety_state_t` and drains its pending safety-ping notification, logs both, decimated - no control math or actuator writes. | Milestones 10-12: rate/attitude control + allocation, writing `MotorOutput`/`ServoOutput`. |
| `RadioTask` | 5 | 20 ms (50 Hz) | Logs its own cadence - no radio transport exists yet. | Milestones 14-15: ESP-NOW and/or RC-receiver `Radio` implementation; deposits setpoints for `FlightControlTask`, non-blocking. |
| `TelemetryTask` | 3 (lowest) | 50 ms (20 Hz), woken by an `esp_timer` (see below) rather than self-paced | Reads `safety_state_t` and logs it, decimated - no downlink transport exists yet. | Milestone 14+: packages state for `RadioTask`'s downlink. |

Priorities are `tskIDLE_PRIORITY + N` (see `task_config.h`); periods were picked from this
document's milestone-1 ranges as follows:

- **`SensorTask`: 500 Hz, the low end of the documented 500 Hz-1 kHz range**, not the high end.
  500 Hz (2 ms) is a clean 2-tick period at this project's 1000 Hz FreeRTOS tick (see
  "Scheduling primitives" below); 1 kHz would need a 1-tick period with zero slack for jitter,
  which is the kind of number to pick after measuring real I2C transaction time on hardware
  (milestone 17), not assert now.
- **`FlightControlTask`: 250 Hz, the low end of the documented 250-500 Hz range**, trailing
  `SensorTask`/`EstimatorTask` deliberately - ESC spin-up and servo slew time already dominate
  actuation latency far more than the control loop's own period does.
- **`RadioTask`: 50 Hz, mid of the documented 20-100 Hz range** - a reasonable placeholder
  between ESP-NOW's and a typical RC receiver's frame rates; the actual number is protocol-
  dependent and will be revisited once milestone 14/15 picks one.
- **`TelemetryTask`: 20 Hz, mid of the documented 10-50 Hz range.**
- **`SafetyTask`: 100 Hz**, as originally sketched - kept at the low end of nothing, since a
  late failsafe is the one deadline miss this project cannot tolerate.

General scheduling principles this table encodes (unchanged from milestone 1, now with real
numbers behind them):

- Priority ordering follows *consequence of a missed deadline*, not just data rate: `SafetyTask`
  outranks everything because a late failsafe is a crash risk, while `TelemetryTask` outranks
  nothing because a late log line is just a late log line.
- `EstimatorTask` is coupled to `SensorTask`'s cadence rather than given an independent timer
  (see "Scheduling primitives" below), to avoid adding artificial latency between "gyro sample
  exists" and "attitude estimate reflects it."
- `RadioTask` and `TelemetryTask` never contend with the control path for CPU: both sit at the
  bottom of the priority ladder, and the one payload handoff wired so far (`SensorTask` ->
  `EstimatorTask`) uses latest-value overwrite semantics precisely so a slow consumer can never
  back-pressure a faster producer.
- All six tasks are placed well below ESP-IDF's own system tasks (e.g. the `esp_timer` dispatch
  task runs at `ESP_TASK_TIMER_PRIO` = `configMAX_PRIORITIES-3` = 22 on this project's 25-level
  scale), so driver-internal and RTOS-infrastructure tasks always preempt application logic when
  they need to.

### Scheduling primitives

Four FreeRTOS primitives are wired up, each for a reason specific to what it connects - not
sprinkled in for coverage:

- **Queue** (`SensorTask` -> `EstimatorTask`): a length-1 queue written via `xQueueOverwrite()`
  every `SensorTask` cycle. Overwrite (not a blocking multi-slot send) because `SensorTask` is
  this project's highest-throughput periodic task short of `SafetyTask` and must never block on
  a slow downstream reader; `EstimatorTask` always gets the freshest sample, never a backlog of
  stale ones. See `firmware/main/sensor_task.h`.
- **Task notification** (`SafetyTask` -> `FlightControlTask`): a bare `xTaskNotifyGive()` /
  `ulTaskNotifyTake()` signal with no payload, sent once per `SafetyTask` cycle. This is
  deliberately separate from the mutex-protected state below: the notification's only job is
  "something changed, you may want to look," which is exactly what a single-word notification is
  for, versus a queue (which would need a payload this doesn't have) or polling the mutex on a
  tighter timer (which would just be busy-waiting). See `firmware/main/safety_task.h`.
- **Timer** (`esp_timer` -> `TelemetryTask`): a periodic `esp_timer` drives `TelemetryTask`'s
  wake instead of `vTaskDelayUntil()`. Telemetry cadence is a pure wall-clock pacing requirement
  independent of any other task's execution - a good fit for `esp_timer`'s own dedicated
  high-priority service task, decoupled from every other task's scheduling, rather than sharing
  `TelemetryTask`'s own delay loop. On plain `esp32` this callback runs in that dedicated
  `esp_timer` task (`ESP_TASK_TIMER_PRIO`), **not** interrupt context - ESP32 only supports
  `esp_timer`'s TASK dispatch method (ISR dispatch is an ESP32-C2-only feature), so the callback
  in `firmware/main/main.c` calls a plain `xTaskNotifyGive()`, not the `FromISR` variant. The
  other five tasks are each periodic in their own right (`SensorTask` by hardware cadence, the
  rest by control/safety timing), so `vTaskDelayUntil()` is the simpler, correct tool there
  instead.
- **Mutex** (`safety_state_t`, `firmware/main/safety_state.c`): the one genuinely cross-task
  shared state in this milestone - `{ armed, checked_at_us }`, written by `SafetyTask` and read
  by `FlightControlTask` and `TelemetryTask`. It needs a mutex specifically because it is a
  *multi-field* struct that must be observed consistently (a reader must never see `armed` from
  one write and `checked_at_us` from a different one - a torn read across the two fields).
  Nothing else in this milestone needs one: the sensor queue already serializes its single
  writer/single reader, and the notification is a single word, atomic by construction. Mutexes
  are not added anywhere else "for consistency" - only where multi-field state is genuinely
  shared.

### Task watchdog

`CONFIG_ESP_TASK_WDT_EN`/`_INIT` (ESP-IDF defaults, unchanged) enable the task watchdog timer
(TWDT) and initialize it automatically at boot. `firmware/sdkconfig.defaults` overrides two
related defaults for this milestone:

- `CONFIG_ESP_TASK_WDT_PANIC=y` (stock default: `n`, log-and-continue). Flight-control firmware
  should not silently keep running after a task hang; a timeout now triggers the panic handler
  (which prints the triggered-task diagnostic via `esp_task_wdt_print_triggered_tasks()`, then
  reboots). This is the closest available safe action until milestone 16 gives `SafetyTask` a
  real motor-shutdown/disarm path - see below for why "just disarm from the WDT's own context"
  isn't the obvious right long-term answer either.
- `CONFIG_ESP_TASK_WDT_TIMEOUT_S=1` (stock default: 5s). This project's slowest task period is
  `TelemetryTask`'s 50ms - 1s is ~20x that, generous margin above normal jitter while catching a
  real hang 5x faster than the stock default.

All six tasks subscribe themselves (`esp_task_wdt_add(NULL)`, called from within each task) and
feed the watchdog every loop iteration (`esp_task_wdt_reset()`), including the four stub/low-
priority tasks: priority governs who wins CPU *contention*, but the watchdog governs *liveness*,
and a hang in `TelemetryTask` (a leaked mutex, an infinite loop) is still a real bug worth
catching and rebooting from even though its own output is never flight-safety-critical.

**Intended eventual behavior, once real actuator output exists (milestone 16+):** a bare panic-
reboot is a safe-enough placeholder now (nothing is armed), but is not the final design. The
natural-looking extension point, `esp_task_wdt_isr_user_handler()`, runs in interrupt context on
the WDT's own trip, and per this document's own ISR-discipline rule (see below) cannot safely
perform I2C/PWM writes to force a motor-shutdown/disarm command from there. The more realistic
real design is a hardware-level failsafe instead - e.g. an ESC/receiver signal-loss timeout, or
`actuators_init_safe()`'s idle/neutral output simply persisting because nothing refreshed it -
rather than software issuing writes from the WDT ISR itself. This nuance is exactly why milestone
16 needs to design the real failsafe path deliberately rather than this milestone inventing one.

### Sensor hardware exercised

`SensorTask` (`firmware/main/sensor_task.c`) calls the real milestone 3/4 driver interfaces
(`mpu6050_read()`/`bmp581_read()`) when `CONFIG_BICOPTER_SENSORS_ENABLED` is set
(`firmware/main/Kconfig.projbuild`) - a new Kconfig option this milestone adds, defaulting to
`n` since no board/pinout has been finalized (see docs/hardware.md). With it disabled (the
default, and the only configuration exercised in this environment), `SensorTask` skips I2C
entirely and publishes an all-invalid `sensor_sample_t` every cycle instead of attempting a bus
that doesn't exist - verified in QEMU (see below), not on hardware, since QEMU's `esp32` machine
does not model attached I2C peripherals either. The `CONFIG_BICOPTER_SENSORS_ENABLED=y` code
path (I2C bus creation, `mpu6050_init()`/`bmp581_init()`, per-cycle reads with graceful failure
handling) was written and reviewed against the driver headers but not exercised at all - same
verified-vs-deferred bar as the drivers themselves in milestone 3/4.

### What was verified this milestone, and how

No physical ESP32 board was available in this environment (same constraint as every milestone so
far - see docs/hardware.md). Verified:

- `idf.py build` succeeds (clean full rebuild, no warnings from this project's code) with all six
  tasks, the queue/notification/timer/mutex plumbing, and the task watchdog wired into `main/`.
- Real QEMU boot (same method as milestone 2/docs/firmware.md): all six tasks start, run
  concurrently, and log at their configured cadences with no crash, no FreeRTOS assertion, and no
  watchdog timeout over a ~12-second run (thousands of cycles across the fastest tasks). Observed
  cycle counts over that window matched their target rates (e.g. `SensorTask` reached cycle
  ~5500, `SafetyTask` ~1100, `TelemetryTask` ~220 - all within a few percent of their 500 Hz/
  100 Hz/20 Hz targets over ~11 real seconds), and `EstimatorTask`'s received-sample count tracked
  `SensorTask`'s cycle count 1:1, confirming the queue handoff. This is real evidence the task/
  queue/notification/timer/watchdog wiring behaves correctly under QEMU's emulated clock - it is
  **not** a substitute for measuring real scheduling behavior (actual I2C bus timing, real
  interrupt latency, real priority-inversion risk under bus contention) on physical hardware,
  which remains milestone 17's job per this document's original scope note above.
- This process also caught a real bug before it shipped: `firmware/sdkconfig` (gitignored,
  regenerated from `sdkconfig.defaults` only when absent) was stale from an earlier milestone's
  build and still carried the 100 Hz stock FreeRTOS tick after `sdkconfig.defaults` was edited to
  request 1000 Hz - `pdMS_TO_TICKS(4)` for `FlightControlTask`'s period rounded down to 0 ticks at
  the stale 100 Hz rate, tripping `vTaskDelayUntil`'s `xTimeIncrement > 0` assertion on the very
  first cycle. Deleting the stale `sdkconfig` and letting `idf.py build` regenerate it from
  `sdkconfig.defaults` fixed it; anyone changing `sdkconfig.defaults` on an existing checkout
  needs to do the same (`rm firmware/sdkconfig && idf.py build`), since ESP-IDF only auto-applies
  defaults when `sdkconfig` doesn't exist yet.
- `tests/task_util_test.c`: the one piece of this milestone's logic pure enough to host-test
  (`SensorTask`'s barometer-read decimation check, `firmware/main/task_util.c`) - everything else
  added this milestone is FreeRTOS/ESP-IDF scaffolding not meaningfully testable off-target.

Not verified, and deferred to milestone 17 along with every other hardware-dependent item this
project has deferred so far: real task timing/jitter under actual I2C bus contention, real
interrupt latency for the MPU6050's data-ready path feeding into `SensorTask`, and the
`CONFIG_BICOPTER_SENSORS_ENABLED=y` code path generally.

## Interrupt context vs. task context

- **Interrupt context (ISR):** limited to what genuinely cannot wait — capturing a hardware
  timestamp or raw sample the instant a peripheral signals data-ready (e.g. an IMU
  data-ready GPIO interrupt, or a PWM/DShot peripheral's completion interrupt), and waking the
  corresponding task (e.g. `xTaskNotifyFromISR`/`xSemaphoreGiveFromISR`). No I2C/SPI
  transactions, no floating-point-heavy math, no dynamic allocation happen in ISR context — all
  of that is deferred to the woken task, per standard FreeRTOS ISR discipline (ISRs must be
  short and non-blocking).
- **Task context:** everything else — bus transactions to read sensor registers, all estimation
  and control math, actuator PWM/protocol writes (setting a duty cycle or building a DShot frame
  is not itself time-critical to the microsecond once the value is computed), radio
  send/receive, telemetry packaging, and all safety-condition evaluation. This is where nearly
  all of the system's logic lives, which is also why the priority/rate table above is the
  primary tool for meeting real-time requirements rather than pushing more work into ISRs.

This split is deliberately conservative: only sample-timestamping and task-waking are
interrupt-driven initially. Milestone 6's task architecture didn't need to revisit it - the one
new timer-driven wake it adds (`esp_timer` -> `TelemetryTask`, see "Scheduling primitives" above)
turned out to run in task context on plain `esp32` anyway, not an ISR. If hardware integration
(milestone 17) finds specific timing requirements an ISR-woken task can't meet, that's the point
where more logic would move into interrupt context — not before, since ISR-context bugs are far
more expensive to debug than task-context ones.
