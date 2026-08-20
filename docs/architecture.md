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
itself is only validated on real hardware (milestone 6).

## FreeRTOS tasks

Target rates and priorities below are starting points to be tuned once real sensor/actuator
timing is measured on hardware (milestone 6); they encode the reasoning, which is the part
worth fixing now.

| Task | Target rate | Relative priority | Why |
|---|---|---|---|
| `SafetyTask` | 100 Hz | Highest | Must preempt everything to enforce disarm/failsafe within one control cycle's latency; cheap to run (mostly comparisons against thresholds/timeouts), so high priority costs little. |
| `SensorTask` | 500 Hz–1 kHz (IMU-driven) | High | Feeds the estimator; IMU gyro integration error grows with sample latency/jitter, so this needs the tightest, most consistent scheduling short of safety itself. Barometer is read far slower (~50 Hz) from within the same task or a lower-rate companion, since baro dynamics are much slower than attitude dynamics. |
| `EstimatorTask` | Matches `SensorTask` (500 Hz–1 kHz) or a fixed-decimation fraction of it | High | Attitude estimate quality depends on tight coupling to fresh gyro samples; should run immediately after each sensor sample is available rather than on an independent timer, to minimize propagation delay into the controller. |
| `FlightControlTask` | 250–500 Hz | High (below `SensorTask`/`EstimatorTask`) | Rate-loop control benefits from running as fast as the estimate updates, but bicopter actuator response (ESC spin-up, servo slew) doesn't need sub-millisecond command updates, so this can trail the estimator slightly without losing authority. |
| `RadioTask` | 20–100 Hz (protocol-dependent: ESP-NOW vs. RC-receiver frame rate) | Medium | Setpoint/telemetry cadence is bounded by the link, not by control dynamics; blocking or jitter here must never stall `FlightControlTask`, hence lower priority and a non-blocking handoff (queue/mailbox) into it. |
| `TelemetryTask` | 10–50 Hz | Low | Downlink logging/telemetry is observational; any delay here is invisible to flight behavior, so it should never contend with control-path tasks for CPU. |

General scheduling principles this table encodes:

- Priority ordering follows *consequence of a missed deadline*, not just data rate: `SafetyTask`
  outranks everything because a late failsafe is a crash risk, while `TelemetryTask` outranks
  nothing because a late log line is just a late log line.
- `EstimatorTask` is coupled to `SensorTask`'s cadence rather than given an independent timer,
  to avoid adding artificial latency between "gyro sample exists" and "attitude estimate
  reflects it."
- `RadioTask` and `TelemetryTask` communicate with the control-path tasks only through
  non-blocking queues/mailboxes (latest-value semantics), so a slow or stalled radio link cannot
  back-pressure the control loop.

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
interrupt-driven initially. If milestone 6 (FreeRTOS task architecture) or later hardware
integration finds specific timing requirements an ISR-woken task can't meet, that's the point
where more logic would move into interrupt context — not before, since ISR-context bugs are far
more expensive to debug than task-context ones.
