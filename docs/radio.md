# Radio (Milestone 14: ESP-NOW; Milestone 15: CRSF RC receiver)

`firmware/components/radio/` implements this project's radio links: ESP-NOW (Milestone 14,
Milestone 6's `RadioTask` stub's first real transport) and CRSF over a UART-connected RC receiver
(Milestone 15, the second concrete `Radio` backend). This document covers the generic `Radio` HAL
interface, both backends' wire formats, their callback/driver/task-context splits,
packet-loss/staleness tracking, pairing/wiring/calibration, and what remains
hardware-unverified. See [architecture.md](architecture.md#hardware-abstraction-layer) for where
`Radio` fits among the project's other HAL interfaces, and [AGENTS.md](../AGENTS.md) for the
directory-layout/testing conventions both milestones follow.

## Scope

- A generic, protocol-independent `Radio` interface (`include/radio.h`), implemented unchanged by
  both backends — see "Milestone 15: no `Radio` interface changes needed" below for confirmation.
- ESP-NOW as the first concrete backend (`esp_now_radio.c/.h`): pairing/configuration, a command
  packet (sequence, timestamp, throttle/roll/pitch/yaw, arm, flight mode), packet validation,
  sequence-number-based stale/reordered-packet rejection, and a minimal one-way telemetry echo.
- CRSF as the second concrete backend (`crsf_radio.c/.h`, `crsf_frame.c/.h`): UART wiring/framing,
  CRC8 validation, RC_CHANNELS_PACKED channel decoding, and configurable channel-to-function
  mapping/calibration — see "Milestone 15: CRSF RC receiver" below.
- Receive-timeout/radio-loss detection, exposed through `Radio`'s health accessors rather than
  left for a future failsafe to compute itself from raw timestamps — identical contract for both
  backends.
- Packet-loss estimation (exact sequence-number gaps for ESP-NOW; a coarser elapsed-time estimate
  for CRSF, which carries no sequence number — see below) and RSSI (real for ESP-NOW when the
  underlying API exposes it; not available for CRSF this milestone, see below).
- Real host-side tests for every host-testable piece of logic (`tests/radio_packet_test.c`,
  `tests/crsf_frame_test.c`).

**Explicitly out of scope, and why:** the failsafe *behavior* that should happen once
`radio_health_t.link_alive` goes false (disarm, hold last command, etc.) is Milestone 16's job —
both milestones' job is making that condition reliably *detectable*, matching the design brief.
Full vehicle-state telemetry (attitude, battery, ...) is left as a documented follow-up; the
telemetry packet Milestone 14 ships is a minimal downlink echo (see "Telemetry" below) so a
ground station can confirm the link is bidirectional, not a substitute for real state telemetry.
CRSF telemetry write-back (receiver → transmitter, real CRSF protocol capability) is likewise out
of scope for Milestone 15 — only RC_CHANNELS_PACKED decoding (receiver → flight controller) is
implemented; see below.

## The `Radio` HAL interface

`include/radio.h` mirrors this project's other HAL interfaces (see
`firmware/components/actuators/include/motor_output.h` for the pattern): a small C "vtable" —
`radio_ops_t` (function pointers) plus an opaque `ctx` — wrapped in one `radio_t` handed out by an
implementation's `*_as_radio()` function (`esp_now_radio_as_radio()` here). No ESP-NOW-specific
type appears in this header.

```c
bool          radio_has_command(radio_t *radio);
esp_err_t     radio_get_command(radio_t *radio, radio_command_t *out_command);
esp_err_t     radio_get_health(radio_t *radio, radio_health_t *out_health);
esp_err_t     radio_deinit(radio_t *radio);
```

`radio_command_t` carries throttle `[0,1]`, roll/pitch/yaw `[-1,1]` (stick-normalized, not yet
mapped to a physical rate/angle setpoint — that mapping is flight-mode logic, a future
`flight_core`/`FlightControlTask` concern this HAL layer deliberately stays out of), `arm`, a raw
`flight_mode` index (semantics owned by future flight-mode logic), plus the packet's `sequence`
and `timestamp_us` for callers that want them.

`radio_health_t` carries `packet_loss_percent`, `last_valid_rx_timestamp_us`, `link_alive`,
`rssi_available`, and `rssi_dbm`. Every field is computed against the implementation's own
notion of "now" at the moment of the call — callers never compute staleness themselves from a raw
timestamp, per the design brief's requirement that link-loss be reliably detectable through the
interface itself.

## ESP-NOW packet format

Both packet types are serialized explicitly byte-by-byte, little-endian
(`firmware/components/radio/src/radio_packet.c`), not a raw `memcpy` of a C struct — this avoids
depending on the sender's and receiver's compilers agreeing on struct packing/alignment (they're
usually the same toolchain on both peers in practice, but a wire format shouldn't rely on that).

Every packet starts with a magic byte (`RADIO_PACKET_MAGIC = 0xB1`) and a version byte
(`RADIO_PACKET_VERSION = 1`), and ends with a one-byte additive checksum over everything before
it. This is **not** a substitute for ESP-NOW's own transport-level integrity check — 802.11's FCS
already guarantees the bytes weren't corrupted in transit. It guards against a different failure
mode: some *other* ESP-NOW device broadcasting on the same channel whose packets happen to reach
this receiver. The magic/version/checksum triple makes it very unlikely a foreign packet is
mistaken for a valid command.

### Command packet (33 bytes)

| Bytes | Field | Notes |
|---|---|---|
| 1 | magic | `0xB1` |
| 1 | version | `1` |
| 4 | sequence | `uint32`, little-endian, monotonically increasing per sender |
| 8 | timestamp_us | `int64`, sender's `esp_timer_get_time()` at construction |
| 4 | throttle | `float32`, `[0,1]` |
| 4 | roll | `float32`, `[-1,1]` |
| 4 | pitch | `float32`, `[-1,1]` |
| 4 | yaw | `float32`, `[-1,1]` |
| 1 | arm | `0`/nonzero |
| 1 | flight_mode | raw index |
| 1 | checksum | additive sum of the preceding 32 bytes, mod 256 |

### Telemetry packet (21 bytes)

A minimal downlink echo, not full vehicle-state telemetry (see "Scope" above): this device's own
outgoing sequence counter, its timestamp, current armed/flight-mode state, and its own
*receive*-side packet-loss estimate (so a ground station watching the downlink can tell both that
it's alive and how the uplink it's sending is faring).

| Bytes | Field | Notes |
|---|---|---|
| 1 | magic | `0xB1` |
| 1 | version | `1` |
| 4 | sequence | `uint32` |
| 8 | timestamp_us | `int64` |
| 1 | armed | `0`/nonzero |
| 1 | flight_mode | raw index |
| 4 | packet_loss_percent | `float32`, this device's receive-side loss estimate |
| 1 | checksum | additive sum of the preceding 20 bytes, mod 256 |

### Validation

`radio_command_packet_parse()`/`radio_telemetry_packet_parse()` reject (return `false`, leave the
output untouched) on: wrong length, bad magic or version, a checksum mismatch, or any non-finite
float field (NaN/Inf) — a defensive check against a corrupted or adversarially-crafted packet that
happened to still pass the checksum. All four checks are covered by
`tests/radio_packet_test.c`.

## Callback vs. task context

This is the milestone's most safety-relevant design point, called out explicitly in the design
brief: **ESP-NOW's receive callback runs in the Wi-Fi driver's own task context — not interrupt
context, but still foreign, high-priority, latency-sensitive context this driver must not do real
work in.** This differs from Milestone 3's MPU6050 driver, whose data-ready handler is a genuine
GPIO ISR (`IRAM_ATTR`, see `firmware/components/sensors/src/mpu6050.c`); ESP-NOW's callback is
plain task-context code (unmarked, like `firmware/main/main.c`'s `telemetry_timer_cb()`), but the
same "minimal work only" discipline from
[architecture.md](architecture.md#interrupt-context-vs-task-context) applies to it because it's
still not *this driver's own* task and must never be blocked or slowed down.

- **Callback context** (`esp_now_recv_cb()`, `esp_now_radio.c`): bounds-checks the received
  length against the raw-packet buffer's fixed size, copies the bytes (and RSSI, if the driver
  exposed it) into a `raw_packet_t`, and posts it to a FreeRTOS queue with a non-blocking
  `xQueueSend(..., 0)`. No packet parsing beyond the length bound-check, no logging, no blocking
  calls. A full queue silently drops the packet rather than blocking the Wi-Fi task.
- **Task context** (`esp_now_radio_process_pending()`, called from `RadioTask` —
  `firmware/main/radio_task.c` — once per 20ms/50Hz cycle): drains the queue and does all real
  work — `radio_command_packet_parse()` (wire-format validation), `radio_seq_is_newer()`
  (staleness/reordering rejection), `radio_loss_tracker_record()` (packet-loss tracking), and
  updating the mutex-protected state `radio_t`'s ops read. This is the "dedicated task" the design
  brief calls for, wired into Milestone 6's existing `RadioTask` rather than a new one — matching
  this project's preference for reusing the six-task architecture over adding tasks per feature.

The mutex around `esp_now_radio_dev`'s command/health state follows the same reasoning as
`firmware/main/safety_state.c`: it's multi-field state (command fields, loss-tracker fields,
last-valid timestamp, RSSI) that a reader must never observe half-updated across two different
writes.

## Sequence-number staleness and reordering

`radio_seq_is_newer(sequence, last_sequence, has_last_sequence)` uses RFC1982-style serial-number
arithmetic — `(int32_t)(sequence - last_sequence) > 0` — so a `uint32` sequence counter wrapping
around after ~4 billion packets is correctly treated as "newer," not rejected as an apparent
rollback. A duplicate or out-of-order (older) sequence is rejected before it ever reaches the
loss tracker or updates the last-known command. `tests/radio_packet_test.c` covers strictly
increasing sequences, duplicates, reordering, and wraparound.

## Packet-loss tracking

`radio_loss_tracker_t` accumulates `received_count` (packets actually recorded) and
`expected_count` (the sum of sequence-number gaps between consecutive *accepted* packets — a run
of N dropped packets between two received ones counts as N, even though neither surrounding
packet was itself rejected). `radio_loss_tracker_loss_percent()` reports
`100 * (1 - received/expected)`, 0% until at least one packet has been recorded. This is an
estimate from the receiver's own point of view (it never sees the packets that never arrived,
only the gaps its sequence numbers imply), which is the best any receiver-side estimate can do
without a two-way acknowledgment protocol.

## Radio-loss detection

`radio_is_stale(last_valid_rx_timestamp_us, now_us, timeout_us)` — deliberately mirroring
`firmware/components/sensors/include/mpu6050_convert.h`'s `mpu6050_is_stale()` shape for
consistency — is a pure function: the caller supplies "now," nothing is tracked internally as
wall-clock state. `radio_t`'s `has_command()`/`get_health()` call it against
`CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS` (default 500ms, 10x `RadioTask`'s own 20ms cycle) every
time they're queried, so `link_alive` always reflects genuinely current staleness rather than a
value computed once and left to go stale itself. **What happens once `link_alive` goes false is
explicitly Milestone 16's job** — this interface only makes the condition detectable, per the
design brief.

## RSSI

ESP-IDF's ESP-NOW receive callback exposes RSSI for real: `esp_now_recv_info_t.rx_ctrl` points at
a `wifi_pkt_rx_ctrl_t` whose `rssi` field (signed, dBm) is populated by the Wi-Fi driver for every
received frame (confirmed against `esp_wifi/include/local/esp_wifi_types_native.h` in this
project's pinned ESP-IDF v5.5.5 checkout). `esp_now_recv_cb()` copies it into the raw packet
alongside the payload; `esp_now_radio_process_pending()` carries it through to
`radio_health_t.rssi_dbm` with `rssi_available = true`. `rssi_available` exists for a future
transport (or an ESP-NOW send-status-only path) that genuinely can't expose RSSI, not because this
one can't — it's real here, not fabricated.

## Pairing / configuration

ESP-NOW has no discovery or negotiated pairing protocol — both peers are configured with each
other's MAC address ahead of time, and must agree on a fixed Wi-Fi channel (ESP-NOW rides on the
Wi-Fi radio without an actual AP association or channel negotiation).

Configured via `firmware/main/Kconfig.projbuild`'s "Bicopter radio (ESP-NOW)" menu:

| Option | Default | Meaning |
|---|---|---|
| `CONFIG_BICOPTER_RADIO_ENABLED` | `n` | Gate, same pattern as `CONFIG_BICOPTER_SENSORS_ENABLED` — off until a real peer is paired |
| `CONFIG_BICOPTER_RADIO_PEER_MAC` | `FF:FF:FF:FF:FF:FF` (placeholder) | The paired peer's MAC, `AA:BB:CC:DD:EE:FF` |
| `CONFIG_BICOPTER_RADIO_WIFI_CHANNEL` | `1` | Must match on both peers |
| `CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS` | `500` | Staleness window for `link_alive` |

**Procedure once real hardware exists:** flash both peers, read each board's Wi-Fi station MAC
(`esp_wifi_get_mac()` or `idf.py monitor`'s boot log), set each board's
`CONFIG_BICOPTER_RADIO_PEER_MAC` to the *other* board's MAC, pick and set the same
`CONFIG_BICOPTER_RADIO_WIFI_CHANNEL` on both, set `CONFIG_BICOPTER_RADIO_ENABLED=y`, and reflash.
No physical boards were available to actually carry this out in this environment — see
"Verified vs. deferred" below.

`esp_now_radio_init()` brings up Wi-Fi station mode with `WIFI_STORAGE_RAM` (ESP-NOW doesn't need
credentials persisted to flash), initializes NVS (`nvs_flash_init()`, required by the Wi-Fi driver
even though this project stores nothing else there yet), starts Wi-Fi, sets the configured
channel, initializes ESP-NOW, registers the receive callback, and adds the configured peer.

Only one `esp_now_radio` instance can exist at a time: `esp_now_register_recv_cb()` takes a bare
function pointer with no per-instance context argument, and there is only one Wi-Fi/ESP-NOW
peripheral on this chip anyway, so this isn't a real limitation in practice.

## Telemetry

`esp_now_radio_send_telemetry()` sends one telemetry packet (see format above) to the paired peer
via `esp_now_send()`. Kept deliberately minimal per this milestone's scope call: a downlink echo
proving the link is bidirectional and giving a ground station the receiver's own loss estimate,
not full vehicle-state telemetry (attitude, altitude, battery, ...) — that's left as a documented
follow-up for whichever later milestone has real state to publish (once `EstimatorTask` is wired
to `flight_core/estimation/`, per `estimation.md`'s "Firmware wiring" section). Nothing in
`firmware/main/` currently calls `esp_now_radio_send_telemetry()` — it exists on the interface and
is exercised only by build-time compilation, not by `RadioTask`'s cycle, since there's no real
telemetry payload to send yet.

## Milestone 15: CRSF RC receiver

`firmware/components/radio/`'s second concrete `Radio` backend: CRSF (Crossfire), decoding RC
stick/switch input from a UART-connected receiver into the same `radio_command_t` shape ESP-NOW
already produces.

### Protocol choice: CRSF over SBUS

The design brief named two well-supported serial RC protocols: CRSF and SBUS. Both were seriously
considered; CRSF was chosen for three reasons, weighed honestly rather than picked by default:

1. **CRSF has a real per-frame CRC8; classic SBUS does not.** This milestone's design brief calls
   for implementing and testing a real checksum/CRC — CRSF's CRC8 (poly 0xD5, "DVB-S2", verified
   in `tests/crsf_frame_test.c` against the published CRC-8/DVB-S2 catalogue check value) satisfies
   that directly. Classic SBUS (the Futaba-originated protocol as implemented by essentially every
   receiver that speaks "SBUS," including non-Futaba brands) carries **no CRC or checksum field at
   all** — only a fixed start byte (`0x0F`) and end byte (`0x00`) delimiting each 25-byte frame.
   This is a real, checkable protocol fact, not a design brief typo we're silently working around:
   stating it plainly here (rather than picking SBUS and quietly not delivering a CRC) is the same
   honesty convention as Milestone 12's no-pitch-authority finding and Milestone 13's
   accelerometer-observability finding — a genuine constraint of the chosen technology, reported
   rather than glossed over.
2. **CRSF is bidirectional**, with real telemetry-return capability (not implemented this
   milestone — see "Explicitly out of scope" below) for future use, on gear that's increasingly
   common on newer builds.
3. **SBUS's traditional downside — inverted UART, usually requiring an external hardware inverter
   on the receiving MCU — is real for many microcontrollers but is *not* actually a hard problem on
   this project's ESP32 target.** ESP-IDF's UART driver exposes `uart_set_line_inverse()` with the
   `UART_SIGNAL_RXD_INV` mask (`~/esp/esp-idf/components/esp_driver_uart/include/driver/uart.h`,
   `hal/uart_types.h`, confirmed against this project's pinned ESP-IDF v5.5.5 checkout), letting
   the UART peripheral invert the RX line in hardware with zero external circuitry. This is a real
   finding worth recording for whoever revisits this choice later (e.g. if a specific receiver on
   hand only speaks SBUS): the usual objection to SBUS on this MCU doesn't hold. It didn't outweigh
   reason 1, since CRSF satisfies the milestone's CRC requirement outright rather than needing a
   documented gap.

**Scope narrowing:** only CRSF's `RC_CHANNELS_PACKED` frame type (`0x16`) is parsed — the frame
type carrying receiver → flight-controller channel data, which is all this milestone needs. Other
real CRSF frame types (link statistics `0x14`, battery `0x08`, GPS `0x02`, and CRSF's own
telemetry-return frames) exist in the protocol but are out of scope here, the same kind of
documented scope-narrowing Milestone 14 applied to ESP-NOW's telemetry (a minimal echo, not full
vehicle-state telemetry). This implementation is receive-only: no CRSF telemetry write-back is
sent.

### CRSF frame format

```
[sync(1)] [length(1)] [type(1)] [payload(N)] [crc8(1)]
```

- **Sync byte**: `0xC8` (`CRSF_SYNC_BYTE`) — the value CRSF receivers commonly use when addressing
  the flight controller, and the byte this project's parser requires as the frame's first byte
  (matching common open-source CRSF implementations, e.g. Betaflight/iNav's own CRSF drivers).
- **Length**: covers `type + payload + crc` (not the sync or length bytes themselves).
- **Type**: `0x16` for `RC_CHANNELS_PACKED`, the only type this implementation parses.
- **Payload** (`RC_CHANNELS_PACKED`, 22 bytes): 16 channels, 11 bits each, packed little-endian
  (channel 0's 11 bits occupy the low bits of byte 0, spilling into byte 1, and so on) — a total
  frame size of 26 bytes for this frame type.
- **CRC8**: DVB-S2 polynomial `0xD5`, initial value `0x00`, no input/output reflection, computed
  over `type + payload` (excluding sync, length, and the CRC byte itself).

`firmware/components/radio/include/crsf_frame.h` / `src/crsf_frame.c` implement this — CRC8,
byte-level frame synchronization (`crsf_frame_sync_push_byte()`, which finds sync bytes, reads the
declared length, accumulates exactly that many bytes, and silently discards garbage before a valid
sync byte), and `crsf_parse_rc_channels_frame()` (sync/length/type/CRC validation plus 11-bit
channel unpacking). Zero ESP-IDF dependency by design — same driver-testing convention as
`radio_packet.c` (Milestone 14) and `mpu6050_convert.c` (Milestone 3, see
[AGENTS.md](../AGENTS.md)). All of it is covered by `tests/crsf_frame_test.c` (140 checks):
CRC8 against the published CRC-8/DVB-S2 check value, valid-frame decoding (including a
distinct-value-per-channel vector specifically chosen to catch an off-by-one in the 11-bit packing
math, not just symmetric all-same-value frames that could hide one), rejection of a wrong sync
byte, a mismatched length byte, the wrong frame type, a corrupted payload byte, and a corrupted CRC
byte, the frame synchronizer's resync-after-garbage and reset-after-completion behavior, an
implausible declared-length rejection, channel-to-command calibration (endpoints, center, clamping
of out-of-calibration-range raw values, flight-mode quantization), an out-of-range configured
channel index reading as 0 instead of indexing out of bounds, and the packet-loss estimate's
behavior (0% before enough data exists, ~0% at the nominal rate, a substantial estimate after a
long gap, disabled when configured with a non-positive nominal period).

### UART driver context: how this differs from Milestone 14's callback split

Milestone 14's write-up above centers on ESP-NOW's receive callback running in foreign (Wi-Fi
driver) task context and needing to be kept minimal by hand. CRSF's story is different in a way
worth stating explicitly: **this driver never writes its own ISR or receive callback at all.**
ESP-IDF's UART driver (`uart_driver_install()`) already does that internally — a driver-owned
interrupt copies bytes from the UART hardware FIFO into a driver-owned RX ring buffer and posts
`uart_event_t` notifications (`UART_DATA`, `UART_FIFO_OVF`, `UART_BUFFER_FULL`, ...) to the
`QueueHandle_t` the driver hands back at install time. That interrupt, and the byte-copying it
does, is ESP-IDF-internal code this project did not write — unlike the MPU6050 driver's GPIO ISR
(`mpu6050.c`) or ESP-NOW's task-context receive callback (`esp_now_radio.c`), both of which this
project wrote and had to keep minimal by hand.

- **Driver-internal context** (ESP-IDF's own UART ISR + ring buffer, not this project's code):
  copies FIFO bytes into the ring buffer, posts `uart_event_t` events to the queue.
- **This driver's task context** (`crsf_radio_process_pending()`, called once per `RadioTask`
  cycle — the same "dedicated task, not a new one" pattern Milestone 14 established): drains the
  event queue non-blockingly, pulls bytes out of the ring buffer via `uart_read_bytes()`, feeds
  them through `crsf_frame_sync_push_byte()`, validates/decodes complete frames via
  `crsf_parse_rc_channels_frame()`, maps channels to a command via `crsf_channels_to_command()`,
  and updates the mutex-protected state `radio_t`'s ops read — all real work, exactly mirroring
  where Milestone 14 put its own real work (never in the callback/ISR).

This is idiomatic ESP-IDF UART usage per the driver's own documentation (interrupt/ring-buffer
mode via `uart_driver_install()`, not a busy-polling `uart_read_bytes()` loop), the pattern this
milestone's design brief asked for.

### Channel mapping and calibration

CRSF's `RC_CHANNELS_PACKED` frame carries 16 raw 11-bit values (0-2047) with no inherent semantics
— which physical channel is "throttle" and what raw range means "0% to 100%" are conventions set
by the transmitter/receiver configuration, not the wire protocol. `crsf_channel_map_t`
(`crsf_frame.h`) makes both configurable rather than hardcoding one vendor's convention:

- **Channel-to-function assignment**: `throttle_channel`, `roll_channel`, `pitch_channel`,
  `yaw_channel`, `arm_channel`, `flight_mode_channel` — each a 0-15 channel index.
- **Endpoint/center calibration**: `raw_min`/`raw_center`/`raw_max` — throttle maps unipolar
  `[0,1]` from `[raw_min, raw_max]`; roll/pitch/yaw map bipolar `[-1,1]` around `raw_center`
  (independently scaled on each side, so an off-center `raw_center` is handled correctly, not
  assumed to bisect `[raw_min, raw_max]`). `arm_threshold` is the raw value at/above which the arm
  channel reads armed. `flight_mode_positions` quantizes the flight-mode channel into that many
  discrete buckets across `[raw_min, raw_max]`.
- All values are clamped to their documented output range even when a raw channel value falls
  outside `[raw_min, raw_max]` (a mis-calibrated transmitter, or a momentary out-of-range value,
  must never produce an out-of-contract `radio_command_t`) — tested in
  `tests/crsf_frame_test.c`.

Configured via `firmware/main/Kconfig.projbuild`'s "Bicopter radio (CRSF RC receiver)" menu:

| Option | Default | Meaning |
|---|---|---|
| `CONFIG_BICOPTER_CRSF_RADIO_ENABLED` | `n` | Gate, same pattern as `CONFIG_BICOPTER_SENSORS_ENABLED`/`CONFIG_BICOPTER_RADIO_ENABLED` — off until a real receiver is wired. **Mutually exclusive with `CONFIG_BICOPTER_RADIO_ENABLED`** (ESP-NOW) — enabling both is a build-time `#error` in `radio_task.c`, confirmed in this environment (see "Verified" below) |
| `CONFIG_BICOPTER_CRSF_UART_PORT` | `1` | ESP32 UART port; UART0 is typically reserved for the console |
| `CONFIG_BICOPTER_CRSF_UART_RX_GPIO` | `16` | Placeholder, unverified against real board wiring |
| `CONFIG_BICOPTER_CRSF_UART_TX_GPIO` | `-1` (disabled) | Unused this milestone (receive-only); reserved for a future telemetry-write-back follow-up |
| `CONFIG_BICOPTER_CRSF_BAUD_RATE` | `420000` | CRSF's common default; some links use 400000/921600/higher |
| `CONFIG_BICOPTER_CRSF_COMMAND_TIMEOUT_MS` | `200` | `link_alive` staleness window — shorter than ESP-NOW's 500ms default since CRSF links commonly run well under 20ms/frame |
| `CONFIG_BICOPTER_CRSF_NOMINAL_FRAME_PERIOD_US` | `4000` | For the packet-loss estimate — see below |
| `CONFIG_BICOPTER_CRSF_THROTTLE_CHANNEL` .. `_FLIGHT_MODE_CHANNEL` | `0`..`5` | Placeholder sequential channel assignment (throttle=0, roll=1, pitch=2, yaw=3, arm=4, flight_mode=5) — arbitrary but explicit, not asserting a real-world standard like "AETR" that could be subtly wrong for the actual TX/RX pair |
| `CONFIG_BICOPTER_CRSF_FLIGHT_MODE_POSITIONS` | `3` | Discrete switch-position count for the flight-mode channel |
| `CONFIG_BICOPTER_CRSF_RAW_MIN` / `_RAW_CENTER` / `_RAW_MAX` | `172` / `992` / `1811` | The de facto CRSF/OpenTX endpoint convention (~988-2012µs mapped onto an 11-bit range) — a reasonable default, not asserted correct for every transmitter |
| `CONFIG_BICOPTER_CRSF_ARM_THRESHOLD` | `992` | Raw value at/above which the arm channel reads armed |

**Calibration procedure once real hardware exists:** bind the receiver to a transmitter, connect
its UART output to the configured RX GPIO (and ground/power per the receiver's datasheet), set
`CONFIG_BICOPTER_CRSF_UART_PORT`/`_RX_GPIO` to match the real wiring, flash with
`CONFIG_BICOPTER_CRSF_RADIO_ENABLED=y`, and observe `RadioTask`'s logged channel/health output
(or a temporary debug log of raw channel values) while moving each stick/switch to its extremes —
read off the actual raw min/center/max for each channel being used and set
`CONFIG_BICOPTER_CRSF_RAW_MIN`/`_RAW_CENTER`/`_RAW_MAX` (and per-channel assignments, if the
receiver's channel order differs from the default) to match. No physical CRSF receiver was
available to carry this out in this environment — see "Verified vs. deferred" below.

### Staleness and packet-loss estimate

`link_alive`/staleness reuses `radio_packet.h`'s `radio_is_stale()` **unchanged** — the exact same
pure function ESP-NOW's `has_command()`/`get_health()` call, just with CRSF's own
`last_valid_rx_timestamp_us` and `CONFIG_BICOPTER_CRSF_COMMAND_TIMEOUT_MS`. This is a real reuse,
not a lookalike reimplementation, so failsafe logic (Milestone 16) built against `link_alive` works
identically regardless of which backend is active, per this milestone's acceptance criteria.

`packet_loss_percent` is necessarily different: CRSF's wire format carries **no sequence number**,
unlike this project's own ESP-NOW packet format (which was deliberately designed with one for
exactly this purpose). `crsf_loss_tracker_t` (`crsf_frame.h`) estimates loss from elapsed wall-clock
time against a configured nominal frame period (`CONFIG_BICOPTER_CRSF_NOMINAL_FRAME_PERIOD_US`)
instead: `expected_frames ≈ elapsed_time / nominal_period + 1`, compared against frames actually
received. This is honestly a coarser estimate than ESP-NOW's exact sequence-gap count — it needs a
roughly-correct nominal period to mean anything, and it's a cumulative average since the first
frame (not a sliding window), so it under-reacts to a loss burst late in a long-running session.
`link_alive`/staleness detection never depends on it. Documented here rather than presented as
being as precise as ESP-NOW's figure.

### RSSI

Not available this milestone: RSSI/link-quality-back-to-the-flight-controller is carried over
CRSF's telemetry-return channel, which this implementation doesn't send (see "Explicitly out of
scope" above and "Scope narrowing" in the protocol-choice section) — unlike ESP-NOW, whose Wi-Fi
driver exposes RSSI on every received frame for free, independent of any application-level
telemetry. `radio_health_t.rssi_available` is always `false` for this backend.

### `Radio` HAL interface: no changes needed

Confirming this milestone's acceptance criterion directly: **`radio.h` required no changes.**
`CrsfRadio`'s `radio_t` ops populate every existing `radio_command_t`/`radio_health_t` field using
the same interface `EspNowRadio` already implements. Two fields are filled differently, which is a
data-availability difference in what each wire protocol carries, not an interface gap:
- `radio_command_t.sequence`: CRSF has no sequence number on the wire, so this backend fills it
  from a local monotonically-increasing frame counter (`crsf_radio_dev::frame_counter`) instead of
  a value carried in the packet.
- `radio_command_t.timestamp_us`: CRSF has no sender-side timestamp field (unlike this project's
  own ESP-NOW packet format), so this backend fills it with the receiver's own
  `esp_timer_get_time()` at the moment the frame was processed, not a sender-side value.

Note also that `crsf_frame.h`'s pure logic deliberately does **not** include `radio.h` and instead
defines its own `crsf_command_t` struct mirroring `radio_command_t`'s
throttle/roll/pitch/yaw/arm/flight_mode fields — `radio.h`'s `radio_t` vtable needs `esp_err_t`
(`esp_err.h`, an ESP-IDF header) for its other members, and pulling that transitively into the
pure/host-testable module would have broken its zero-ESP-IDF-dependency property for the sake of
one shared struct shape. `crsf_radio.c` (which already depends on ESP-IDF for the UART driver)
does the trivial field-by-field conversion into a real `radio_command_t` — the same pattern
`esp_now_radio.c` already uses to convert `radio_command_packet_t` into one. This mirrors
`radio_packet.h`'s own precedent (`radio_command_packet_t`, a separate type from
`radio_command_t` for the identical reason) rather than being a new decision.

## Verified vs. deferred

Same honesty convention as every prior hardware-dependent milestone (see `docs/hardware.md`): no
physical ESP32 boards or RC transmitter/receiver hardware were available in this environment, so
nothing about actual over-the-air/over-the-wire link behavior — real packet delivery, real RSSI
values, real latency, real range, two ESP-NOW boards actually pairing, a real CRSF receiver's raw
channel values/timing — was exercised.

**Verified (Milestone 14, ESP-NOW):**
- `idf.py build` succeeds both with `CONFIG_BICOPTER_RADIO_ENABLED=n` (default) and with it set to
  `y` (a placeholder peer MAC/channel) — the second build genuinely compiles and links
  `esp_now_radio.c` against ESP-IDF v5.5.5's real `esp_wifi`/`esp_now`/`nvs_flash`/`esp_netif`/
  `esp_event` headers and libraries (confirmed via a real `idf.py build` in this environment, not
  review alone — `esp-idf/esp_wifi/` and the `radio` component's object files are visibly built
  and linked into `bicopter_fc.elf` in that configuration).
- Every piece of pure logic (`radio_packet.c`: command/telemetry serialize+parse round-trips,
  malformed/undersized/corrupted-packet rejection, sequence-number staleness/reordering including
  wraparound, packet-loss-percentage computation, staleness timeout arithmetic) has real
  passing host-side tests (`tests/radio_packet_test.c`, 47 checks, `ctest --test-dir tests/build`)
  that run identically on the host and would run identically on-target, since this code has zero
  ESP-IDF dependency by construction.
- `esp_now_radio.c`'s use of the ESP-NOW/Wi-Fi API (`esp_now_init()`, `esp_now_register_recv_cb()`,
  `esp_now_add_peer()`, `esp_now_send()`, `esp_wifi_set_channel()`, the
  `esp_now_recv_info_t`/`wifi_pkt_rx_ctrl_t` RSSI field) was written and reviewed against ESP-IDF
  v5.5.5's actual installed headers in this environment (`~/esp/esp-idf/components/esp_wifi/
  include/`), not from memory or an assumed API shape.

**Not verified (Milestone 14), deferred to Milestone 17 (hardware integration) alongside every
other hardware-dependent item this project has deferred so far:**
- Real ESP-NOW packet delivery between two physical boards, real range/latency, real RSSI values
  under actual RF conditions.
- Whether `CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS`'s 500ms default is an appropriate staleness
  window for real transmitter frame rates and real-world packet loss — this is exactly the kind of
  number that should be tuned against measured link behavior, not asserted now (same reasoning
  `docs/architecture.md` already applies to `RadioTask`'s own 50Hz period).

**Verified (Milestone 15, CRSF):**
- `idf.py build` succeeds with `CONFIG_BICOPTER_CRSF_RADIO_ENABLED=n` (default), with it set to `y`
  (a placeholder UART port/pins/baud) — the second build genuinely compiles and links
  `crsf_radio.c` against ESP-IDF v5.5.5's real `driver`/`esp_driver_uart` UART API (confirmed via a
  real `idf.py build` in this environment, not review alone). Also confirmed: enabling **both**
  `CONFIG_BICOPTER_RADIO_ENABLED` and `CONFIG_BICOPTER_CRSF_RADIO_ENABLED` fails the build at
  `radio_task.c`'s `#error` guard with a clear message, rather than silently picking one backend.
- Every piece of pure logic (`crsf_frame.c`: CRC8 against the published CRC-8/DVB-S2 check value,
  `RC_CHANNELS_PACKED` decoding including a distinct-value-per-channel vector, rejection of a bad
  sync byte/length mismatch/wrong type/corrupted payload/corrupted CRC, the frame synchronizer's
  garbage-discard and resync-after-completion behavior, channel-to-command calibration and
  clamping, flight-mode quantization, out-of-range channel-index safety, and the packet-loss
  estimate) has real passing host-side tests (`tests/crsf_frame_test.c`, 140 checks,
  `ctest --test-dir tests/build`) that run identically on the host and would run identically
  on-target, since this code has zero ESP-IDF dependency by construction.
- `crsf_radio.c`'s use of ESP-IDF's UART driver API (`uart_param_config()`, `uart_set_pin()`,
  `uart_driver_install()` in interrupt/ring-buffer mode, `uart_read_bytes()`,
  `uart_flush_input()`, the `uart_event_t`/`UART_DATA`/`UART_FIFO_OVF`/`UART_BUFFER_FULL` event
  types) was written and reviewed against ESP-IDF v5.5.5's actual installed headers in this
  environment (`~/esp/esp-idf/components/esp_driver_uart/include/driver/uart.h`), not from memory
  or an assumed API shape. The CRC8/DVB-S2 algorithm and the RC_CHANNELS_PACKED 16x11-bit
  little-endian packing were cross-checked against an independent Python reference implementation
  (used to generate `tests/crsf_frame_test.c`'s frame byte vectors), not hand-derived and
  hand-packed with no independent check.

**Not verified (Milestone 15), deferred to Milestone 17 alongside every other hardware-dependent
item this project has deferred so far:**
- Real CRSF frame reception from a physical transmitter/receiver pair — actual raw channel
  values/ranges for a real TX/RX, actual frame rate/jitter, actual UART signal integrity over a
  real wiring harness.
- Whether the placeholder channel-to-function assignment (`CH0..CH5` = throttle/roll/pitch/yaw/
  arm/flight_mode) and endpoint calibration (`172`/`992`/`1811`) match the real transmitter's
  configuration — see the "Calibration procedure" above for what to do once hardware exists.
- Whether `CONFIG_BICOPTER_CRSF_COMMAND_TIMEOUT_MS`'s 200ms default and
  `CONFIG_BICOPTER_CRSF_NOMINAL_FRAME_PERIOD_US`'s 4ms default are appropriate for the real
  transmitter's configured CRSF update rate — both are placeholders pending real hardware
  measurement, same reasoning as ESP-NOW's 500ms timeout above.
- The UART wiring itself (RX/TX GPIO assignment, power/ground) on a real board, since no board is
  chosen yet (see `docs/hardware.md`).
- The pairing procedure above, end to end, on real hardware.
