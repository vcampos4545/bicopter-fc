# Control

**Stub.** No control code exists yet. This document will be filled in as the control milestones
land, not before — the bicopter force/torque model and control-allocation math must be derived
from the vehicle's actual geometry (motor positions, servo tilt-axis offsets, arm lengths, thrust
curves), not invented placeholder mixing coefficients. Writing that math against unknown
geometry now would just have to be redone.

## What will live here

As milestones 10–12 (see [TODO.md](../TODO.md)) land, this document will cover:

- **Rate (angular-velocity) controller** — structure (PID or equivalent), inputs/outputs, and
  tuning approach. Milestone 10.
- **Attitude controller** — how attitude error becomes a rate setpoint, and how it interacts
  with the rate controller (cascaded loop). Milestone 11.
- **Bicopter force/torque model** — the actual geometric derivation of how each motor's thrust
  (modulated by its servo tilt angle) contributes to net body force and torque, given this
  vehicle's specific motor/servo placement. This is a prerequisite for allocation, not
  allocation itself.
- **Control allocation** — the derivation (not assertion) of the mapping from desired body
  torque + thrust to per-motor throttle and per-servo tilt angle, inverted from the force/torque
  model above, including how it handles the actuator limits documented in
  [hardware.md](hardware.md). Milestone 12.

## Interim conventions fixed now

To keep later milestones consistent with the rest of the repository, two things are fixed here
even though the math isn't:

- All control quantities use the body-frame and units convention from [README.md](../README.md)
  (SI units, X=forward/Y=right/Z=down).
- Control allocation output is expressed through the `MotorOutput`/`ServoOutput` hardware-
  abstraction interfaces described in [architecture.md](architecture.md), so the same allocation
  code runs unmodified on `firmware/` and in `simulator/`.
