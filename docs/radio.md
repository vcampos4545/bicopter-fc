# Radio (Milestone 14: ESP-NOW)

`firmware/components/radio/` implements this project's first real radio link: ESP-NOW,
Milestone 6's `RadioTask` stub's first real transport. This document covers the generic `Radio`
HAL interface, the ESP-NOW packet format, the callback/task-context split, packet-loss/staleness
tracking, pairing/configuration, and what remains hardware-unverified. See
[architecture.md](architecture.md#hardware-abstraction-layer) for where `Radio` fits among the
project's other HAL interfaces, and [AGENTS.md](../AGENTS.md) for the directory-layout/testing
conventions this milestone follows.

## Scope

- A generic, protocol-independent `Radio` interface (`include/radio.h`) that Milestone 15's
  conventional-RC-receiver implementation will also implement, so `RadioTask` and any future
  consumer never depend on ESP-NOW specifics.
- ESP-NOW as the first concrete backend (`esp_now_radio.c/.h`): pairing/configuration, a command
  packet (sequence, timestamp, throttle/roll/pitch/yaw, arm, flight mode), packet validation,
  sequence-number-based stale/reordered-packet rejection, and a minimal one-way telemetry echo.
- Receive-timeout/radio-loss detection, exposed through `Radio`'s health accessors rather than
  left for a future failsafe to compute itself from raw timestamps.
- Packet-loss estimation (sequence-number gaps) and RSSI (real, when the underlying ESP-NOW API
  exposes it — see below).
- Real host-side tests for every host-testable piece of logic (`tests/radio_packet_test.c`).

**Explicitly out of scope, and why:** the failsafe *behavior* that should happen once
`radio_health_t.link_alive` goes false (disarm, hold last command, etc.) is Milestone 16's job —
this milestone's job is making that condition reliably *detectable*, matching the design brief.
Full vehicle-state telemetry (attitude, battery, ...) is left as a documented follow-up; the
telemetry packet this milestone ships is a minimal downlink echo (see "Telemetry" below) so a
ground station can confirm the link is bidirectional, not a substitute for real state telemetry.
Milestone 15's RC-receiver `Radio` implementation is unstarted; this milestone's job was making
sure `radio.h` doesn't need to change shape when that lands.

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

## Verified vs. deferred

Same honesty convention as every prior hardware-dependent milestone (see `docs/hardware.md`): no
physical ESP32 boards were available in this environment, so nothing about the actual over-the-air
link — real packet delivery, real RSSI values, real latency, real range, two boards actually
pairing — was exercised.

**Verified:**
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

**Not verified, deferred to Milestone 17 (hardware integration) alongside every other
hardware-dependent item this project has deferred so far:**
- Real ESP-NOW packet delivery between two physical boards, real range/latency, real RSSI values
  under actual RF conditions.
- Whether `CONFIG_BICOPTER_RADIO_COMMAND_TIMEOUT_MS`'s 500ms default is an appropriate staleness
  window for real transmitter frame rates and real-world packet loss — this is exactly the kind of
  number that should be tuned against measured link behavior, not asserted now (same reasoning
  `docs/architecture.md` already applies to `RadioTask`'s own 50Hz period).
- The pairing procedure above, end to end, on real hardware.
