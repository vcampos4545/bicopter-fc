# Firmware: toolchain, build, and boot verification

Milestone 2 deliverable: `firmware/` is a real ESP-IDF project that boots. This document
records the toolchain version, how to reproduce the build, and how boot was verified.

## Toolchain

- **ESP-IDF version:** v5.5.5 (the current stable release branch as of this milestone).
- **Target chip:** plain `esp32` (pinned in `firmware/sdkconfig.defaults` via
  `CONFIG_IDF_TARGET`). No specific dev board or pinout is assumed — see
  [hardware.md](hardware.md).

## Installing ESP-IDF

Standard Espressif layout, following the
[official get-started guide](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/):

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.5 --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32
```

`install.sh` also fetches the `qemu-xtensa` tool (used for boot verification below) as part of
the standard toolchain install.

Each new shell needs the environment sourced before `idf.py` is on `PATH`:

```sh
source ~/esp/esp-idf/export.sh
```

## Building

```sh
cd firmware
idf.py build          # target is already pinned by sdkconfig.defaults; no set-target needed
```

`idf.py build` succeeds with no warnings introduced by this project's own code (verified via a
full clean rebuild). Confirmed image size (Milestone 2's minimal `main/` only, no drivers):

```
Flash Code   61702 bytes (.text)
Flash Data   37256 bytes (.rodata + .appdesc)
IRAM         52847 / 131072 bytes (40.32%)
DRAM         12508 / 180736 bytes (6.92%)
Total image: 162097 bytes
```

To target a fresh checkout explicitly (equivalent to what `sdkconfig.defaults` already
provides): `idf.py set-target esp32`.

## Flashing to real hardware (not done in this milestone)

No physical ESP32 board was available in this environment. Once one is:

```sh
idf.py -p <PORT> flash monitor
```

This is deferred to a later milestone (per `docs/hardware.md`, the specific board/pinout isn't
chosen yet, so there is nothing to flash to beyond a generic "esp32" target).

## Boot verification: QEMU

ESP-IDF v5.5 ships `idf.py qemu` (no longer `--preview`) backed by the `qemu-xtensa` tool
installed by `install.sh`. The combined `idf.py qemu monitor` action requires an interactive
TTY for its serial monitor (it errored with `Monitor requires standard input to be attached to
TTY` in this non-interactive environment), so boot was verified by building the same QEMU flash
image `idf.py qemu` builds (`build/qemu_flash.bin`, `build/qemu_efuse.bin`) and invoking
`qemu-system-xtensa` directly with serial output redirected to a file instead of the
interactive monitor:

```sh
idf.py qemu           # builds build/qemu_flash.bin + build/qemu_efuse.bin, boots once, exits
# or, to capture serial output non-interactively without a TTY:
qemu-system-xtensa -M esp32 -m 4M \
  -drive file=build/qemu_flash.bin,if=mtd,format=raw \
  -drive file=build/qemu_efuse.bin,if=none,format=raw,id=efuse \
  -global driver=nvram.esp32.efuse,property=drive,value=efuse \
  -global driver=timer.esp32.timg,property=wdt_disable,value=true \
  -nic user,model=open_eth -nographic -monitor none \
  -serial file:/tmp/qemu_serial.log
```

This produced a real boot: the 2nd-stage bootloader, FreeRTOS `main_task` starting, and the
`main/` component's startup log all appear. Captured log (trimmed to the relevant tail):

```
I (691) boot: ESP-IDF v5.5.5 2nd stage bootloader
I (692) boot: Multicore bootloader
...
I (1479) app_init: Project name:     bicopter_fc
I (1480) app_init: ESP-IDF:          v5.5.5
...
I (1511) spi_flash: detected chip: winbond
I (1527) main_task: Started on CPU0
I (1527) main_task: Calling app_main()
I (1527) bicopter-fc: bicopter-fc firmware starting (ESP-IDF v5.5.5)
I (1527) bicopter-fc: boot: no sensors/actuators/radio initialized yet (milestone 2 scope)
I (1527) bicopter-fc: bicopter-fc ready
```

This is a genuine emulated boot (bootloader → partition table → app image load → FreeRTOS task
start → `app_main()` log lines), not just a successful compile. It is honest QEMU verification,
not real-hardware verification — no physical board was available in this environment (see
above).

## What's deferred to later milestones

- Real-hardware flash/boot (needs a physical ESP32 board; not available here).
- Actuator (ESC/servo PWM) or radio (ESP-NOW/RC) driver code or initialization, and wiring any
  driver — including the MPU6050 driver added in Milestone 3
  (`firmware/components/sensors/`) — into `main/`. `main/` still only boots FreeRTOS and logs;
  per `AGENTS.md`, the boot sequence is deliberately `BOOT -> (nothing else armed yet) -> log
  ready`, and task-architecture wiring is Milestone 6's job, not this one.
- Component registration for `firmware/components/{actuators,radio,estimation,control,safety,
  telemetry}/` — each stays a `.gitkeep`'d stub until its own milestone adds real code and a
  `CMakeLists.txt` (`sensors/` is real as of Milestone 3).
- Board-specific pin mapping / Kconfig — deferred until a specific board is chosen (see
  [hardware.md](hardware.md)); only the chip target (`esp32`) is configured so far.
