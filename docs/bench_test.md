# Bench-test tooling (Milestone 18)

Milestone 18 is the last purely-software milestone before Milestone 17 (hardware integration),
which genuinely requires physical hardware in hand and remains a captain-directed milestone once
that hardware exists (see [TODO.md](../TODO.md)). This document covers what Milestone 18 built to
make that eventual hardware bring-up safe: three build-time operating modes, a `BENCH_TEST` build
configuration with a compile-time motor-disable guarantee, and a UART bench-test console with
independent sensor/servo/ESC/radio test commands. It closes with the recommended real-hardware
bring-up sequence Milestone 17 is meant to actually run against.

See [AGENTS.md](../AGENTS.md) for the directory-layout/driver-testing conventions this milestone
follows, and [docs/safety.md](safety.md) for the flight-mode state machine and arming/failsafe
logic this milestone's operating modes sit alongside (and deliberately do not duplicate or
bypass).

## The three operating modes

`firmware/main/Kconfig.projbuild`'s "Bicopter operating mode" menu adds a single
`BICOPTER_OPERATING_MODE` Kconfig choice, mutually exclusive, selecting exactly one of:

- **`SIMULATION`** (default) - the safe default for day-to-day firmware development, CI, and QEMU
  boot verification. Nothing about this mode assumes real actuator hardware is attached;
  `CONFIG_BICOPTER_ACTUATORS_ENABLED` normally stays off alongside it, the same "no board chosen
  yet" gating every other not-yet-wired hardware option in this project uses (sensors, radio,
  battery). The bench-test console's read-only sensor/radio streaming commands still work in this
  mode (they only ever read existing state); servo/esc_test commands work only if actuators are
  separately enabled, and esc_test additionally requires `HARDWARE_TEST` (below). Not to be
  confused with `simulator/` (the desktop physics simulator, a completely separate CMake target
  that never links `firmware/`) - this mode governs `firmware/`'s own behavior and does not itself
  simulate anything.
- **`HARDWARE_TEST`** - for bringing up real hardware on the bench, per this document's
  recommended sequence below. This is the only mode in which the bench-test console's `esc_test`
  command (motor spin) is reachable at all - the command handler in
  `firmware/main/bench_test_task.c` is itself wrapped in `#if CONFIG_BICOPTER_OPERATING_MODE_HARDWARE_TEST`,
  so in any other mode the code path that could ever call `motor_output_write()` from the console
  does not exist in the compiled binary, not just "isn't reached at runtime." The normal flight
  arm/throttle path (`firmware/components/safety/`, see [docs/safety.md](safety.md)) is completely
  unaffected by this choice - `esc_test` is a second, independent, more tightly gated way to
  command a motor, not a shortcut through the first. Combine with
  `CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED` (below) for a belt-and-suspenders bench session
  where even a fully confirmed `esc_test` cannot spin a motor.
- **`FLIGHT`** - normal operation: the arming/failsafe state machine, and, once a future milestone
  wires it in, the real control-allocation actuator path (see AGENTS.md's "What's real vs. stub
  right now" - `FlightControlTask` still doesn't call a real allocator as of this milestone). The
  bench-test console's `servo` and `esc_test` commands are both refused in this mode - bypassing
  the control loop or spinning a motor from a diagnostic console has no legitimate use in a flight
  build, and once the control-allocation path lands, a console command writing to the same
  actuator handles concurrently would be actively dangerous. Read-only sensor/radio streaming
  commands remain available for in-flight diagnostics.

### Why build-time, not boot-time-latched

The mode is a Kconfig choice, resolved entirely at compile time - selecting a different mode means
reflashing a different binary, not sending a runtime command. This was chosen over a boot-time-
latched value (e.g. read once from NVS or a GPIO strap at startup and held for the rest of the
boot) for one reason: a build-time choice cannot be affected by anything that happens after boot,
which rules out an entire class of mode-confusion risk a boot-time-latched value cannot - a
corrupted NVS read, a miswired strap pin floating into the wrong state, or (if the value were ever
made runtime-settable instead of merely boot-latched) a stray/replayed radio packet or a bug
flipping the running vehicle into `HARDWARE_TEST`'s relaxed motor-test path mid-flight. The
tradeoff is operational inconvenience - switching modes requires a reflash - which this project
judges the correct price for the guarantee, per the design brief's overall bias toward structural,
not procedural, safety (the same reasoning behind `BENCH_TEST`'s compile-time motor disable
below, and behind [docs/safety.md](safety.md)'s "no partial-credit" arming preconditions). A
boot-time-latched design was considered and rejected specifically because it reintroduces a
runtime code path (reading and trusting some boot-time signal) that a pure Kconfig choice avoids
entirely.

## `BENCH_TEST`: the compile-time motor-disable guarantee

`CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED` (default `n`) is a separate Kconfig option from the
operating mode above - it can be combined with any of the three modes, though the intended real
usage is alongside `HARDWARE_TEST` for the safest possible bench session (servos and sensors live,
motors physically incapable of spinning even with a fully confirmed `esc_test`).

**How it's enforced.** `firmware/components/actuators/include/pwm_esc_bench_test.h` translates the
Kconfig value into a plain, always-defined macro (`PWM_ESC_BENCH_TEST_MOTORS_DISABLED`) so it's
visible outside `sdkconfig.h` too (see "What this does and doesn't prove" below).
`firmware/components/actuators/src/pwm_esc_output.c` - the ESC driver, the *only* file in this
project that ever calls ESP-IDF's LEDC PWM API for a motor channel - wraps every LEDC call in
`#if PWM_ESC_BENCH_TEST_MOTORS_DISABLED`:

- `pwm_esc_output_init()` skips `ledc_timer_config()`/`ledc_channel_config()` entirely in a
  BENCH_TEST build - the motor GPIO is never configured as a PWM output at all, not just left at
  an idle duty cycle.
- `pwm_esc_apply_pulse_us()` - the single function in the whole driver that ever calls
  `ledc_set_duty_and_update()` - has its real body (the call) replaced with a no-op `return ESP_OK`
  in a BENCH_TEST build. Every public entry point that could drive a duty cycle
  (`pwm_esc_output_write()`, `pwm_esc_output_set_idle()`) routes through this one function, so
  gating it here is sufficient regardless of what throttle value any caller - a confirmed
  `esc_test` console command, or a real control-allocation path once one is wired - requests.
- `pwm_esc_output_deinit()` skips `ledc_stop()` (nothing was ever started).

This is a genuine `#if` exclusion, not a runtime `if` on a value that could be mis-set: in a
BENCH_TEST build, the object code for `pwm_esc_output.c` contains **zero references** to any
`ledc_*` function. Verified directly for this milestone (not just asserted): building the
`actuators` component with `CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=y` and running
`xtensa-esp32-elf-nm -u` on the resulting `pwm_esc_output.c.obj` lists no undefined `ledc_*`
symbols at all, where the same command against a normal build lists all four
(`ledc_channel_config`, `ledc_set_duty_and_update`, `ledc_stop`, `ledc_timer_config`). Tilt servos
are unaffected - `servo_output.c` has no BENCH_TEST gate, since this guarantee is specifically
about motors, per the design brief.

**What this does and doesn't prove.** `pwm_esc_output.c` depends on ESP-IDF's `driver/ledc.h` and
`driver/gpio.h`, so - per AGENTS.md's driver-testing convention, the same split every hardware-I/O
file in this project follows - it is not itself host-testable; it is reviewed, and (this
milestone) verified once by direct `nm` inspection of the real compiled object, as described
above. `tests/pwm_esc_bench_test_gate_test.c` and `tests/pwm_esc_bench_test_gate_disabled_test.c`
(the same source file, built twice with different compile definitions) are real, host-run CTest
checks, but they only prove that `pwm_esc_bench_test.h`'s macro translation is correct under both
configurations - i.e. that `PWM_ESC_BENCH_TEST_MOTORS_DISABLED` resolves to 0 by default and to 1
when the Kconfig option is set. That translation is exactly what `pwm_esc_output.c`'s `#if`
branches on, so it is the load-bearing piece a host test *can* reach; the claim that
`pwm_esc_output.c` itself correctly uses that macro (rather than, say, misspelling it) rests on
code review plus the one-time `nm` verification above, not on an automated test that runs on every
build - the same honest verified-vs-deferred split every other hardware-I/O file in this project
reports.

## Bench-test console: UART command interface

**Interface choice.** A minimal custom UART command parser
(`firmware/components/bench_test/`) was chosen over ESP-IDF's `esp_console` component. The
deciding factor: this project's established driver-testing convention (AGENTS.md) keeps pure
logic hand-written and host-tested rather than delegated to a library, and the same reasoning
applies here - `bench_test_command.c` (parsing/validating a typed line into a command, including
the safety-relevant `esc_test` confirmation-token check) is plain, ESP-IDF-free C, compiled
directly into a host CTest binary (`tests/bench_test_command_test.c`, 66 checks) the same way
`radio_packet.c`/`crsf_frame.c`/`flight_mode.c` already are. `esp_console` would have put that
logic (argument parsing, at minimum) behind a library this project doesn't review or test itself.
`bench_test_console.c` is the ESP-IDF-dependent half: UART driver I/O in interrupt/ring-buffer
mode (`uart_driver_install()`), the same pattern `firmware/components/radio/src/crsf_radio.c`
already established for a UART-connected peripheral - `bench_test_console_process_pending()`
drains the driver's event queue non-blockingly, mirroring
`crsf_radio_process_pending()`/`esp_now_radio_process_pending()`'s drain pattern.

**UART port.** Defaults to UART2 (`CONFIG_BICOPTER_BENCH_TEST_CONSOLE_UART_PORT`), deliberately
distinct from UART0 (logging/programming) and CRSF's own default of UART1 (see the CRSF radio
Kconfig menu), so the console can run concurrently with logging and, if ever needed, a CRSF
receiver without contending for the same UART. RX/TX GPIOs and baud rate are also
Kconfig-configurable - placeholders, not verified against any actual board wiring, same status as
every other pin default in this project (see [docs/hardware.md](hardware.md)).

**Task.** `BenchTestTask` (`firmware/main/bench_test_task.c`) owns the console and dispatches
parsed commands. It is the seventh FreeRTOS task, lowest priority of all seven (below
`TelemetryTask`), and deliberately **not** registered with the task watchdog and given no fixed
`vTaskDelayUntil` period the way this project's six core tasks are (see
[docs/architecture.md](architecture.md#freertos-tasks)) - it is an interactive diagnostic tool,
not part of the real-time flight-control path, and its `esc_test` handler intentionally blocks for
the configured test duration (see below), a duration a strict watchdog period would fight rather
than help.

### Commands

Each command is independently usable without any of the others working first, per this milestone's
requirement - sensor/radio streaming only ever read state something else already publishes, and
servo/`esc_test` only need actuators enabled, not radio or sensors.

| Command | What it does | Gating |
|---|---|---|
| `help` | Prints the command list. | None. |
| `sensor start` / `sensor stop` | Prints the latest IMU/barometer reading (peeked from `SensorTask`'s existing sample queue, the same `xQueuePeek()` pattern `safety_task.c` already uses) once per `BenchTestTask` cycle (10Hz) while streaming is on. | None - works even with `CONFIG_BICOPTER_SENSORS_ENABLED=n` (prints the honest all-invalid stub `SensorTask` publishes in that case). |
| `radio start` / `radio stop` | Prints the latest decoded `RadioCommand` and link health (via `radio_state.h`) once per cycle while streaming is on. | None - independent of arming state, and works with no radio transport enabled (reports "no transport enabled"). |
| `servo <unit 0\|1> <angle_deg>` | Commands one tilt servo directly via `servo_output_write()`, once, independent of the normal control loop. | Refused in a `FLIGHT` build. Reports "actuators not initialized" if `CONFIG_BICOPTER_ACTUATORS_ENABLED=n` or `actuators_init_safe()` failed. |
| `esc_test <unit 0\|1> <throttle 0.0-1.0> CONFIRM` | Commands one motor to a low, clamped throttle for a fixed duration, then automatically returns it to idle. | Only reachable in a `HARDWARE_TEST` build (see above) - the handler itself is `#if`-excluded otherwise. Requires the literal, case-sensitive, exact trailing `CONFIRM` token - any other or missing token parses as `BENCH_TEST_PARSE_MISSING_CONFIRMATION`, not a normal command. Every invocation - confirmed or not - logs a loud `ESP_LOGW` warning naming the unit and throttle, per the design brief's explicit requirement. |

`esc_test`'s extra confirmation is the "distinct, deliberate confirmation... not just the normal
arm/throttle path" this milestone's brief requires: it is not the flight arm sequence
(`firmware/components/safety/`), it is not a stick position, and it cannot be muscle-memory-typed
alongside a normal command the way a single character or "y" could be. Two further layers on top
of the confirmation token itself:

- **Throttle ceiling.** `CONFIG_BICOPTER_BENCH_TEST_ESC_MAX_THROTTLE_PERCENT` (default 15%) hard-
  clamps whatever throttle an operator types, independent of the ESC driver's own
  `min_throttle`/`max_throttle` clamp - a typo or an aggressive test value can never exceed this
  ceiling.
- **Automatic idle.** `CONFIG_BICOPTER_BENCH_TEST_ESC_TEST_DURATION_MS` (default 2000ms): the
  handler commands the clamped throttle, blocks for exactly this long, then unconditionally calls
  `motor_output_set_idle()` and logs that it did. A confirmed `esc_test` can never leave a motor
  spinning indefinitely because an operator forgot a second "stop" command.

### Actuator wiring this milestone added

The bench-test console's `servo`/`esc_test` commands need real `motor_output_t`/
`servo_output_handle_t` handles to command. `actuators_init_safe()`
(`firmware/components/actuators/include/actuators_init.h`) has existed since Milestone 5 but
was, per AGENTS.md, deliberately not called from `firmware/main/` - it needed a per-board GPIO/PWM
config that didn't exist without a chosen board. This milestone adds that config as
`CONFIG_BICOPTER_ACTUATORS_ENABLED` (default `n`, the same "no board chosen yet" gating pattern
`CONFIG_BICOPTER_SENSORS_ENABLED`/`CONFIG_BICOPTER_RADIO_ENABLED`/`CONFIG_BICOPTER_BATTERY_ENABLED`
all use) plus placeholder GPIO/pulse-width/angle-range Kconfig values (see
`firmware/main/Kconfig.projbuild`'s "Bicopter actuator hardware" menu), and calls
`actuators_init_safe()` from `main()` - at the exact call site AGENTS.md's Milestone 5/6 entries
already documented as the natural one (before any control-path task starts, so every motor/servo
is idle/neutral first) - only when that option is set. The result is published via
`firmware/main/actuators_state.h`/`.c` for `BenchTestTask` to read; while disabled, the console
commands report "actuators not initialized" rather than attempting to drive a GPIO nothing is
connected to. This is the same "no board chosen yet, real wiring deferred, but the code path is
real and ready" status every other optional-hardware component in this project has held since its
own introducing milestone.

## Recommended real-hardware bring-up sequence (for Milestone 17)

This is what Milestone 17 (hardware integration - captain-directed, requires physical ESP32/
MPU6050/BMP581/ESC/servo/battery hardware in hand, see [TODO.md](../TODO.md)) is meant to actually
run against. Per the top-level design brief's safety guidance:

1. **Remove propellers** before any motor-related step below. Do not reattach them until every
   step through 7 has been completed and verified correct.
2. **Test sensors before actuators.** Flash a `SIMULATION`-mode build with
   `CONFIG_BICOPTER_SENSORS_ENABLED=y` and the real I2C pins configured. Connect at the bench-test
   console's UART and run `sensor start`; confirm `imu.valid`/`baro.valid` read true and that
   accelerometer/gyro/pressure/temperature values are physically plausible (e.g. accelerometer
   magnitude near 1g at rest, gyro near zero at rest) before touching any actuator wiring.
3. **Test radio independent of arming.** With a radio transport enabled
   (`CONFIG_BICOPTER_RADIO_ENABLED` or `CONFIG_BICOPTER_CRSF_RADIO_ENABLED`), run `radio start` and
   confirm stick/switch movements are reflected correctly (right sign, right channel, `arm`
   toggling as expected) - this works regardless of flight mode or arming state, so it can be
   verified before actuators are even wired.
4. **Test servos before motors.** Reflash with `CONFIG_BICOPTER_OPERATING_MODE_HARDWARE_TEST=y`
   and `CONFIG_BICOPTER_ACTUATORS_ENABLED=y` (motors still physically safe: props are off per step
   1). Run `servo 0 <angle>` and `servo 1 <angle>` across the full configured range and **verify
   servo direction** - a positive commanded angle must move the tilt mechanism the direction this
   project's body-frame convention (README.md/AGENTS.md: X=forward, Y=right, Z=down) predicts, not
   just "moves somewhere."
5. **Test motors, propellers still off.** With props still removed, run `esc_test <unit> <low
   throttle> CONFIRM` for each motor unit individually. **Verify motor direction** (CW/CCW per the
   vehicle's configured spin direction, `VehicleParams::MotorParams` in `flight_core/vehicle/`) by
   observation, not assumption. Confirm the motor returns to idle automatically at the end of the
   configured test duration without a second command.
6. **Verify control signs.** Once sensors, radio, servo direction, and motor direction are all
   individually confirmed, this is the point to cross-check them against each other: does a
   physical tilt of the vehicle in the positive-roll direction produce the sign of gyro/accel
   reading this project's estimator (`flight_core/estimation/`) expects? Does a positive commanded
   servo/motor differential produce the torque direction `flight_core/control/`'s
   `ControlAllocator` derivation (see [docs/control_allocation.md](control_allocation.md)) assumes?
   Getting this wrong is a classic way for a "correctly wired" vehicle to fly incorrectly.
7. **Verify failsafe behavior before any powered flight test.** With the vehicle disarmed, exercise
   each failsafe condition [docs/safety.md](safety.md) documents (radio loss, low/critical battery
   if wired, excessive attitude/rate if an estimator is available) and confirm the configured
   response (default: motor shutdown/disarm) actually happens - do this on the bench, not for the
   first time in the air.
8. **Reattach propellers only after 1-7 are all verified correct.** First powered test with
   propellers attached must be on a **restrained/tethered vehicle** (per the design brief), never a
   free first flight - this project has never flown, and steps 1-7 reduce but do not eliminate the
   chance something is still wired or signed incorrectly.

Steps 2-7 above are exactly what this milestone's tooling (`SIMULATION`/`HARDWARE_TEST` modes,
`BENCH_TEST`'s motor-disable guarantee, and the four bench-test console commands) exists to make
possible without needing the full flight stack wired and without the risk an unrestricted motor
test would otherwise carry.

## Verified vs. deferred

- `idf.py build` succeeds (confirmed in this environment, real ESP-IDF v5.5.5 builds, not review
  alone) for: the default configuration (`SIMULATION`, all optional hardware off);
  `HARDWARE_TEST` + `CONFIG_BICOPTER_ACTUATORS_ENABLED=y`; the same combination plus
  `CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=y` (the intended safest bench-session combination);
  and `FLIGHT` mode.
- The BENCH_TEST motor-disable guarantee was verified directly by inspecting the compiled
  `pwm_esc_output.c.obj` (`xtensa-esp32-elf-nm -u`) under both configurations - see above for the
  exact result.
- `bench_test_command.c`'s parsing/validation logic (including the `esc_test` confirmation-token
  check) has 66 passing host-side checks (`tests/bench_test_command_test.c`); the
  `pwm_esc_bench_test.h` gate-value translation has 2 passing host-side checks under both
  configurations (`tests/pwm_esc_bench_test_gate_test.c`,
  `tests/pwm_esc_bench_test_gate_disabled_test.c`).
- No physical ESP32/MPU6050/BMP581/ESC/servo/battery hardware was available in this environment
  (same constraint as every milestone through 16 - see [docs/hardware.md](hardware.md)). The UART
  console's real byte-level I/O (`bench_test_console.c`), the actuator LEDC hardware calls
  themselves, and the recommended bring-up sequence above are reviewed and, where noted, built
  successfully, but not exercised against real hardware - that is exactly Milestone 17's job, which
  this milestone's tooling exists to support, not preempt.
