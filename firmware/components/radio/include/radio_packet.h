// Pure, hardware-independent ESP-NOW packet logic: wire-format serialize/parse (with validation),
// sequence-number-based staleness/reordering detection, and packet-loss-percentage tracking.
// Nothing in this header or its .c file touches Wi-Fi, ESP-NOW, or any ESP-IDF header, so it
// compiles and runs identically on the target and on the host (see
// tests/radio_packet_test.c) -- same split convention as
// firmware/components/sensors/include/mpu6050_convert.h (see AGENTS.md's driver-testing
// convention): the hardware I/O (esp_now_radio.c) can't be unit-tested without silicon, but the
// packet math it depends on can be, and is, here.
//
// Wire format is serialized explicitly byte-by-byte (little-endian), not a raw struct memcpy --
// this avoids relying on the sender's and receiver's compilers agreeing on struct packing/
// alignment/endianness, which is exactly the kind of assumption that's cheap to get wrong across
// a wire and expensive to debug after the fact.
#ifndef BICOPTER_RADIO_RADIO_PACKET_H
#define BICOPTER_RADIO_RADIO_PACKET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Single byte identifying this project's ESP-NOW application-layer protocol, distinct from
// ESP-NOW's own over-the-air framing/CRC. ESP-NOW only guarantees the bytes it delivered weren't
// corrupted in transit and came from some ESP-NOW sender on the same channel -- it says nothing
// about whether that sender is speaking *this* packet format (e.g. a neighboring, unrelated
// ESP-NOW device on the same channel). The magic byte plus the checksum below are this project's
// own guard against exactly that, not a substitute for ESP-NOW's transport-level integrity check.
#define RADIO_PACKET_MAGIC 0xB1
#define RADIO_PACKET_VERSION 1

// One decoded command packet: sequence number, sender timestamp, throttle/roll/pitch/yaw, arm,
// flight mode -- exactly the design brief's field list. Values are already in the units
// radio_command_t (radio.h) uses; no further conversion happens above this layer.
typedef struct {
    uint32_t sequence;
    int64_t timestamp_us;
    float throttle;
    float roll;
    float pitch;
    float yaw;
    bool arm;
    uint8_t flight_mode;
} radio_command_packet_t;

// magic(1) + version(1) + sequence(4) + timestamp_us(8) + throttle/roll/pitch/yaw(4*4) + arm(1)
// + flight_mode(1) + checksum(1) = 33 bytes.
#define RADIO_COMMAND_PACKET_WIRE_SIZE 33

// Serializes `packet` into `out_buf` (must be at least RADIO_COMMAND_PACKET_WIRE_SIZE bytes).
// Returns the number of bytes written (RADIO_COMMAND_PACKET_WIRE_SIZE), or 0 if buf_len is too
// small.
size_t radio_command_packet_serialize(const radio_command_packet_t *packet, uint8_t *out_buf,
                                       size_t buf_len);

// Validates and parses a received command packet. Rejects (returns false, leaves *out_packet
// untouched) on: wrong length, bad magic/version, checksum mismatch, or a non-finite float field
// (NaN/Inf) -- any of which mean either a malformed/foreign packet or a corrupted one that
// happened to survive ESP-NOW's own transport-level check. `bool arm` decodes as true for any
// nonzero byte.
bool radio_command_packet_parse(const uint8_t *data, size_t len, radio_command_packet_t *out_packet);

// Optional telemetry packet sent back to the paired peer: this device's own outgoing sequence
// number, its timestamp, current armed/flight-mode state, and its *receive*-side packet-loss
// estimate (so a ground station can see the downlink is alive and how the uplink it's sending is
// faring). Minimal by design -- see docs/radio.md for why a fuller telemetry payload (attitude,
// battery, ...) is left as a documented follow-up rather than built out this milestone.
typedef struct {
    uint32_t sequence;
    int64_t timestamp_us;
    bool armed;
    uint8_t flight_mode;
    float packet_loss_percent;
} radio_telemetry_packet_t;

// magic(1) + version(1) + sequence(4) + timestamp_us(8) + armed(1) + flight_mode(1) +
// packet_loss_percent(4) + checksum(1) = 21 bytes.
#define RADIO_TELEMETRY_PACKET_WIRE_SIZE 21

size_t radio_telemetry_packet_serialize(const radio_telemetry_packet_t *packet, uint8_t *out_buf,
                                         size_t buf_len);
bool radio_telemetry_packet_parse(const uint8_t *data, size_t len,
                                   radio_telemetry_packet_t *out_packet);

// True if `sequence` should be accepted as newer than `last_sequence` (RFC1982-style serial
// number arithmetic: `(int32_t)(sequence - last_sequence) > 0`, so it correctly handles uint32
// wraparound rather than treating a wrapped-around sequence as always-older). Always true if
// `has_last_sequence` is false (nothing received yet, so anything is "newer"). Equal sequences
// (a duplicate/replayed packet) are rejected.
bool radio_seq_is_newer(uint32_t sequence, uint32_t last_sequence, bool has_last_sequence);

// Running packet-loss estimate derived from sequence-number gaps among *accepted* packets (i.e.
// packets that already passed radio_seq_is_newer() -- duplicates/reordered packets are rejected
// before reaching this tracker, not filtered here). expected_count grows by the full gap between
// consecutive accepted sequence numbers, so a run of dropped packets between two received ones is
// counted as loss even though neither surrounding packet was itself rejected.
typedef struct {
    bool has_last_sequence;
    uint32_t last_sequence;
    uint32_t received_count;
    uint32_t expected_count;
} radio_loss_tracker_t;

void radio_loss_tracker_reset(radio_loss_tracker_t *tracker);

// Records one accepted packet's sequence number. Caller must only pass sequences that already
// passed radio_seq_is_newer() against this tracker's own last_sequence.
void radio_loss_tracker_record(radio_loss_tracker_t *tracker, uint32_t sequence);

// 0.0 if no packets have been recorded yet; otherwise 100 * (1 - received/expected).
float radio_loss_tracker_loss_percent(const radio_loss_tracker_t *tracker);

// True if `now_us - last_valid_rx_timestamp_us` exceeds `timeout_us` -- mirrors
// firmware/components/sensors/include/mpu6050_convert.h's mpu6050_is_stale() naming/shape for
// consistency. Callers (esp_now_radio's has_command()/get_health()) supply "now" themselves
// rather than this module tracking wall-clock time, keeping it a pure function.
bool radio_is_stale(int64_t last_valid_rx_timestamp_us, int64_t now_us, int64_t timeout_us);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_RADIO_RADIO_PACKET_H
