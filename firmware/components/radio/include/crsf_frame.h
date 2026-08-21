// Pure, hardware-independent CRSF (Crossfire) protocol logic: CRC8 (DVB-S2), byte-level frame
// synchronization, RC_CHANNELS_PACKED frame validation/decoding, channel-to-radio_command_t
// calibration/mapping, and a link-quality estimate. Nothing in this header or its .c file touches
// a UART peripheral or any ESP-IDF header, so it compiles and runs identically on the target and
// on the host (see tests/crsf_frame_test.c) -- same split convention as
// firmware/components/radio/include/radio_packet.h (Milestone 14) and
// firmware/components/sensors/include/mpu6050_convert.h (Milestone 3): the hardware I/O
// (crsf_radio.c) can't be unit-tested without silicon, but the frame/channel math it depends on
// can be, and is, here.
//
// Protocol choice (CRSF over SBUS -- see docs/radio.md for the full writeup): both are viable
// per the design brief. CRSF was chosen primarily because it carries a real per-frame CRC8 (this
// milestone's explicit checksum/CRC requirement) -- classic SBUS, by contrast, has no CRC or
// checksum field at all, only a start/end byte pair, a real finding worth stating plainly rather
// than glossing over. CRSF is also bidirectional (telemetry return, not implemented this
// milestone -- see below) and increasingly common on modern gear. SBUS's traditional downside
// (inverted UART, usually needing an external hardware inverter) is in fact *not* a real problem
// on this project's ESP32 target -- ESP-IDF's UART driver exposes uart_set_line_inverse() with
// UART_SIGNAL_RXD_INV, letting the peripheral invert RX in hardware with no external circuit
// (confirmed against this project's pinned ESP-IDF v5.5.5 headers) -- but that advantage didn't
// outweigh CRSF actually satisfying the milestone's CRC requirement outright.
//
// Scope: only CRSF's RC_CHANNELS_PACKED frame type (0x16, the receiver-to-flight-controller
// channel-data frame) is implemented. Other CRSF frame types (link statistics 0x14, battery 0x08,
// GPS 0x02, and CRSF's own telemetry-return frames) are real parts of the protocol but out of
// scope here, the same kind of documented scope-narrowing Milestone 14 applied to ESP-NOW's
// telemetry (a minimal echo, not full vehicle-state telemetry).
#ifndef BICOPTER_RADIO_CRSF_FRAME_H
#define BICOPTER_RADIO_CRSF_FRAME_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sync byte CRSF receivers use when addressing the flight controller (CRSF's own
// CRSF_ADDRESS_FLIGHT_CONTROLLER value); widely used in practice as the frame sync byte for
// RC_CHANNELS_PACKED frames regardless of the sender's own logical address, matching common
// open-source CRSF implementations (Betaflight, iNav).
#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#define CRSF_NUM_CHANNELS 16
#define CRSF_RC_CHANNELS_PAYLOAD_SIZE 22 // 16 channels * 11 bits / 8 bits-per-byte
// sync(1) + length(1) + type(1) + payload(22) + crc(1)
#define CRSF_RC_CHANNELS_FRAME_SIZE (3 + CRSF_RC_CHANNELS_PAYLOAD_SIZE + 1)
// Generous upper bound on any CRSF frame this project might ever see/discard while resyncing;
// well above CRSF_RC_CHANNELS_FRAME_SIZE (26).
#define CRSF_MAX_FRAME_SIZE 64

// CRC8/DVB-S2: poly 0xD5, init 0x00, no input/output reflection. Verified against the published
// CRC-8/DVB-S2 check value (crsf_crc8((uint8_t*)"123456789", 9) == 0xBC) in
// tests/crsf_frame_test.c.
uint8_t crsf_crc8(const uint8_t *data, size_t len);

// One decoded RC_CHANNELS_PACKED frame's raw 11-bit channel values (0-2047), before any
// endpoint/center calibration.
typedef struct {
    uint16_t raw[CRSF_NUM_CHANNELS];
} crsf_rc_channels_t;

// Validates and decodes one complete RC_CHANNELS_PACKED frame (sync byte through CRC byte,
// inclusive -- exactly CRSF_RC_CHANNELS_FRAME_SIZE bytes, as produced by
// crsf_frame_sync_push_byte() below). Rejects (returns false, leaves *out_channels untouched) on:
// wrong length, bad sync byte, a length byte that disagrees with frame_len, wrong frame type, or
// a CRC8 mismatch.
bool crsf_parse_rc_channels_frame(const uint8_t *frame, size_t frame_len,
                                   crsf_rc_channels_t *out_channels);

// Byte-level frame synchronizer: feed raw UART bytes one at a time. Finds the sync byte, reads
// the declared length byte, and accumulates exactly that many more bytes before reporting a
// complete frame -- garbage bytes before a valid sync byte are silently discarded. This only
// establishes frame *boundaries*; the accumulated bytes still need CRC/type validation via
// crsf_parse_rc_channels_frame() (or a future frame-type-specific parser) before being trusted.
typedef struct {
    uint8_t buf[CRSF_MAX_FRAME_SIZE];
    size_t len; // bytes accumulated so far; 0 means "not synced / between frames"
} crsf_frame_sync_t;

void crsf_frame_sync_reset(crsf_frame_sync_t *sync);

// Feeds one byte. Returns true and sets *out_frame_len when a complete frame (matching its own
// declared length) has been accumulated in sync->buf[0..*out_frame_len) -- the sync state is
// reset for the next frame in the same call, so the caller must consume sync->buf before feeding
// another byte. Returns false (out_frame_len untouched) while still accumulating or resyncing.
bool crsf_frame_sync_push_byte(crsf_frame_sync_t *sync, uint8_t byte, size_t *out_frame_len);

// Maps CRSF's 16 raw channels onto a local plain-C struct matching radio_command_t's
// throttle/roll/pitch/yaw/arm/flight_mode shape (radio.h) -- the same downstream data shape
// Milestone 14's ESP-NOW implementation produces. This module deliberately does NOT include
// radio.h and defines crsf_command_t itself instead, mirroring radio_packet.h's own precedent
// (radio_command_packet_t): radio.h's radio_t vtable needs esp_err_t (esp_err.h, an ESP-IDF
// header), and pulling that in here would break this module's zero-ESP-IDF-dependency,
// host-testable-by-design property for the sake of one shared struct shape. crsf_radio.c (which
// already depends on ESP-IDF for the UART driver) does the trivial field-by-field conversion into
// a real radio_command_t, the same way esp_now_radio.c converts radio_command_packet_t into one.
//
// Both the channel-to-function assignment (which physical channel index drives
// throttle/roll/pitch/yaw/arm/flight_mode) and the endpoint/center calibration (raw value range
// varies by transmitter/receiver brand -- see docs/radio.md's calibration procedure) are
// configurable here, not hardcoded to one vendor's convention.
typedef struct {
    uint8_t throttle_channel;     // channel index (0-15) mapped to unipolar [0,1] throttle
    uint8_t roll_channel;         // channel index (0-15) mapped to bipolar [-1,1] roll
    uint8_t pitch_channel;        // channel index (0-15) mapped to bipolar [-1,1] pitch
    uint8_t yaw_channel;          // channel index (0-15) mapped to bipolar [-1,1] yaw
    uint8_t arm_channel;          // channel index (0-15) thresholded into crsf_command_t.arm
    uint8_t flight_mode_channel;  // channel index (0-15) quantized into crsf_command_t.flight_mode
    uint16_t raw_min;             // raw value at the low stick/switch endpoint
    uint16_t raw_center;          // raw value at stick center (roll/pitch/yaw only)
    uint16_t raw_max;             // raw value at the high stick/switch endpoint
    uint16_t arm_threshold;       // arm_channel raw value at/above which arm reads true
    uint8_t flight_mode_positions; // number of discrete switch positions to quantize into
                                    // (0 or 1 => flight_mode is always 0)
} crsf_channel_map_t;

// Mirrors radio_command_t's (radio.h) throttle/roll/pitch/yaw/arm/flight_mode fields exactly, in
// the same units -- deliberately a separate local type rather than radio_command_t itself; see
// the comment above this section for why.
typedef struct {
    float throttle;       // [0, 1]
    float roll;            // [-1, 1]
    float pitch;            // [-1, 1]
    float yaw;                // [-1, 1]
    bool arm;
    uint8_t flight_mode;
} crsf_command_t;

// Fills out_command's fields from channels, per map. An out-of-range channel index in map (>=
// CRSF_NUM_CHANNELS) is treated as reading 0 rather than indexing out of bounds. Does not (and,
// being radio_command_t-independent, cannot) set sequence/timestamp_us -- CRSF's wire format
// carries neither field (no sequence number, no sender timestamp), unlike this project's own
// ESP-NOW packet format; the caller (crsf_radio.c) fills those from its own local frame counter
// and esp_timer_get_time() when it copies this struct's fields into a real radio_command_t.
void crsf_channels_to_command(const crsf_rc_channels_t *channels, const crsf_channel_map_t *map,
                               crsf_command_t *out_command);

// Cumulative-since-first-frame packet-loss estimate: CRSF carries no sequence number, so unlike
// radio_packet.h's radio_loss_tracker_t (which counts exact sequence-number gaps), this estimates
// loss from elapsed wall-clock time against a configured nominal frame period -- expected frame
// count = elapsed_time / nominal_period + 1, compared against frames actually received. This is
// necessarily an approximation (it needs a roughly-correct nominal_frame_period_us to mean
// anything, and it's a cumulative average since the first frame, not a sliding window, so it
// under-reacts to a loss burst late in a long-running session) -- documented honestly in
// docs/radio.md rather than presented as being as precise as ESP-NOW's sequence-gap figure.
// link_alive/staleness detection (radio_is_stale(), reused unchanged from radio_packet.h) never
// depends on this estimate.
typedef struct {
    bool has_first_frame;
    int64_t first_frame_timestamp_us;
    int64_t last_frame_timestamp_us;
    uint32_t received_count;
    int64_t nominal_frame_period_us;
} crsf_loss_tracker_t;

void crsf_loss_tracker_reset(crsf_loss_tracker_t *tracker, int64_t nominal_frame_period_us);
void crsf_loss_tracker_record(crsf_loss_tracker_t *tracker, int64_t now_us);
// 0.0 until at least two frames have been recorded (nothing to estimate an interval from yet), or
// if nominal_frame_period_us is non-positive (estimation disabled).
float crsf_loss_tracker_loss_percent(const crsf_loss_tracker_t *tracker);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_RADIO_CRSF_FRAME_H
