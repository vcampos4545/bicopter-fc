// Real automated tests for the radio component's pure CRSF logic (crsf_frame.c): CRC8/DVB-S2
// against a published reference vector, RC_CHANNELS_PACKED decoding (including a distinct-value
// vector so each channel's position in the packed bitstream is actually exercised, not just
// symmetric all-same-value data), rejection of corrupted/malformed/wrong-type frames, the
// byte-level frame synchronizer's resync-after-garbage behavior, channel-to-crsf_command_t
// calibration mapping, and the elapsed-time packet-loss estimate. Same hand-rolled harness as
// tests/radio_packet_test.c and tests/mpu6050_convert_test.c (a host-side CTest binary, no test
// framework dependency). Everything in firmware/components/radio/src/crsf_radio.c (the actual
// ESP-IDF UART driver calls) is hardware-dependent and not host-testable -- see docs/radio.md's
// verified-vs-deferred section.
//
// Test frame byte vectors below were generated and cross-checked with an independent Python
// reference implementation of CRC8/DVB-S2 and CRSF's 16x11-bit little-endian bit-packing (not
// hand-packed), to avoid the tests and the implementation sharing the same packing bug.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "crsf_frame.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                       \
    do {                                                                                        \
        g_checks++;                                                                             \
        if (!(cond)) {                                                                          \
            g_failures++;                                                                       \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                                \
        }                                                                                        \
    } while (0)

#define CHECK_NEAR_F(a, b, tol, msg) CHECK(fabsf((a) - (b)) <= (tol), msg)

// --- CRC8/DVB-S2 ---------------------------------------------------------------------------------

static void test_crc8_matches_published_check_value(void)
{
    // The CRC catalogue's standard check value for CRC-8/DVB-S2 (poly 0xD5, init 0x00, no
    // reflection) over the ASCII string "123456789" is 0xBC.
    const uint8_t input[] = "123456789";
    CHECK(crsf_crc8(input, 9) == 0xBC, "crc8 matches the published CRC-8/DVB-S2 check value");
}

static void test_crc8_empty_and_single_byte(void)
{
    CHECK(crsf_crc8(NULL, 0) == 0x00, "crc8 of zero bytes is 0");
    uint8_t single = 0x00;
    CHECK(crsf_crc8(&single, 1) == 0x00, "crc8([0x00]) == 0x00");
    single = 0xFF;
    CHECK(crsf_crc8(&single, 1) == 0xF9, "crc8([0xFF]) == 0xF9 (independently computed)");
}

// --- RC_CHANNELS_PACKED decoding ------------------------------------------------------------------

// sync(0xC8) + len(0x18) + type(0x16) + 22-byte payload + crc; 16 channels all set to
// [172,992,1811,992,992,992,172,1811,992,992,992,992,992,992,992,992].
static const uint8_t kMixedFrame[CRSF_RC_CHANNELS_FRAME_SIZE] = {
    0xC8, 0x18, 0x16, 0xAC, 0x00, 0xDF, 0xC4, 0xC1, 0x07, 0x3E, 0xF0, 0xB1, 0x62,
    0xE2, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0x9E,
};

// All 16 channels zero.
static const uint8_t kZeroFrame[CRSF_RC_CHANNELS_FRAME_SIZE] = {
    0xC8, 0x18, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xEF,
};

// All 16 channels at the max 11-bit value (2047).
static const uint8_t kMaxFrame[CRSF_RC_CHANNELS_FRAME_SIZE] = {
    0xC8, 0x18, 0x16, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x8F,
};

// Channel i = 100 + i*50 -- a distinct value per channel, so decoding a channel at the wrong
// bit offset (an off-by-one in the 11-bit packing) would be caught, unlike the symmetric vectors
// above.
static const uint8_t kDistinctFrame[CRSF_RC_CHANNELS_FRAME_SIZE] = {
    0xC8, 0x18, 0x16, 0x64, 0xB0, 0x04, 0x32, 0xF4, 0xC1, 0x12, 0xAF, 0x40, 0x46,
    0x38, 0xF4, 0x31, 0x11, 0x96, 0x14, 0xC5, 0x2B, 0x77, 0x81, 0x4C, 0x6A, 0x38,
};
static const uint16_t kDistinctExpected[CRSF_NUM_CHANNELS] = {
    100, 150, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850,
};

static void test_parse_valid_mixed_frame(void)
{
    crsf_rc_channels_t channels;
    bool ok = crsf_parse_rc_channels_frame(kMixedFrame, sizeof(kMixedFrame), &channels);
    CHECK(ok, "a valid RC_CHANNELS_PACKED frame parses");
    const uint16_t expected[CRSF_NUM_CHANNELS] = { 172,  992, 1811, 992, 992, 992, 172, 1811,
                                                     992, 992,  992, 992, 992, 992, 992, 992 };
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        CHECK(channels.raw[i] == expected[i], "decoded channel value matches expected");
    }
}

static void test_parse_zero_and_max_frames(void)
{
    crsf_rc_channels_t channels;
    CHECK(crsf_parse_rc_channels_frame(kZeroFrame, sizeof(kZeroFrame), &channels),
          "all-zero frame parses");
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        CHECK(channels.raw[i] == 0, "all-zero frame decodes every channel to 0");
    }

    CHECK(crsf_parse_rc_channels_frame(kMaxFrame, sizeof(kMaxFrame), &channels),
          "all-max frame parses");
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        CHECK(channels.raw[i] == 0x7FF, "all-max frame decodes every channel to 2047 (11 bits)");
    }
}

static void test_parse_distinct_frame_catches_bit_offset_errors(void)
{
    crsf_rc_channels_t channels;
    CHECK(crsf_parse_rc_channels_frame(kDistinctFrame, sizeof(kDistinctFrame), &channels),
          "distinct-value frame parses");
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        CHECK(channels.raw[i] == kDistinctExpected[i],
              "each channel decodes to its own distinct expected value");
    }
}

static void test_parse_rejects_wrong_length(void)
{
    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(kMixedFrame, sizeof(kMixedFrame) - 1, &channels),
          "one byte short of the wire size is rejected");
    CHECK(!crsf_parse_rc_channels_frame(kMixedFrame, sizeof(kMixedFrame) + 1, &channels),
          "one byte too long is rejected");
    CHECK(!crsf_parse_rc_channels_frame(kMixedFrame, 0, &channels), "zero-length frame is rejected");
}

static void test_parse_rejects_bad_sync_byte(void)
{
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
    memcpy(frame, kMixedFrame, sizeof(frame));
    frame[0] = 0xEE; // not CRSF_SYNC_BYTE

    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(frame, sizeof(frame), &channels),
          "a frame with the wrong sync byte is rejected");
}

static void test_parse_rejects_length_byte_mismatch(void)
{
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
    memcpy(frame, kMixedFrame, sizeof(frame));
    frame[1] = 0x10; // declared length no longer matches frame_len - 2

    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(frame, sizeof(frame), &channels),
          "a frame whose length byte disagrees with the buffer length is rejected");
}

static void test_parse_rejects_wrong_frame_type(void)
{
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
    memcpy(frame, kMixedFrame, sizeof(frame));
    frame[2] = 0x14; // CRSF_FRAMETYPE_LINK_STATISTICS, not RC_CHANNELS_PACKED
    // Recompute CRC over the mutated type+payload so this test isolates the type check, not the
    // CRC check.
    uint8_t crc = crsf_crc8(&frame[2], sizeof(frame) - 3);
    frame[sizeof(frame) - 1] = crc;

    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(frame, sizeof(frame), &channels),
          "a well-formed, correctly-CRC'd frame of the wrong type is rejected");
}

static void test_parse_rejects_corrupted_payload(void)
{
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
    memcpy(frame, kMixedFrame, sizeof(frame));
    frame[10] ^= 0xFF; // flip a payload byte without fixing up the CRC

    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(frame, sizeof(frame), &channels),
          "a corrupted payload with a stale CRC is rejected");
}

static void test_parse_rejects_corrupted_crc_byte(void)
{
    uint8_t frame[CRSF_RC_CHANNELS_FRAME_SIZE];
    memcpy(frame, kMixedFrame, sizeof(frame));
    frame[sizeof(frame) - 1] ^= 0xFF; // corrupt only the CRC byte itself

    crsf_rc_channels_t channels;
    CHECK(!crsf_parse_rc_channels_frame(frame, sizeof(frame), &channels),
          "a frame with a corrupted CRC byte (payload otherwise valid) is rejected");
}

// --- frame synchronizer ----------------------------------------------------------------------

static void test_frame_sync_accumulates_one_frame(void)
{
    crsf_frame_sync_t sync;
    crsf_frame_sync_reset(&sync);

    bool got_frame = false;
    size_t frame_len = 0;
    for (size_t i = 0; i < sizeof(kMixedFrame); i++) {
        bool complete = crsf_frame_sync_push_byte(&sync, kMixedFrame[i], &frame_len);
        if (i + 1 < sizeof(kMixedFrame)) {
            CHECK(!complete, "frame is not reported complete before the last byte arrives");
        } else {
            got_frame = complete;
        }
    }
    CHECK(got_frame, "frame is reported complete on the last byte");
    CHECK(frame_len == sizeof(kMixedFrame), "reported frame_len matches the pushed frame's size");
    CHECK(memcmp(sync.buf, kMixedFrame, frame_len) == 0,
          "accumulated bytes match the pushed frame exactly");
}

static void test_frame_sync_discards_garbage_before_sync_byte(void)
{
    crsf_frame_sync_t sync;
    crsf_frame_sync_reset(&sync);

    const uint8_t garbage[] = { 0x00, 0x01, 0xFF, 0x7E, 0x10 };
    for (size_t i = 0; i < sizeof(garbage); i++) {
        size_t frame_len;
        CHECK(!crsf_frame_sync_push_byte(&sync, garbage[i], &frame_len),
              "garbage bytes before a sync byte never complete a frame");
    }

    bool got_frame = false;
    size_t frame_len = 0;
    for (size_t i = 0; i < sizeof(kMixedFrame); i++) {
        got_frame = crsf_frame_sync_push_byte(&sync, kMixedFrame[i], &frame_len);
    }
    CHECK(got_frame, "a real frame following discarded garbage still completes correctly");
    CHECK(frame_len == sizeof(kMixedFrame), "post-garbage frame_len is still correct");
    CHECK(memcmp(sync.buf, kMixedFrame, frame_len) == 0,
          "post-garbage accumulated bytes match the real frame, not the garbage");
}

static void test_frame_sync_resets_after_completing_a_frame(void)
{
    crsf_frame_sync_t sync;
    crsf_frame_sync_reset(&sync);
    size_t frame_len;
    for (size_t i = 0; i < sizeof(kMixedFrame); i++) {
        crsf_frame_sync_push_byte(&sync, kMixedFrame[i], &frame_len);
    }
    CHECK(sync.len == 0, "sync state resets to empty immediately after a completed frame");

    // Feed a second, different frame right after -- confirms the synchronizer is ready to frame
    // again, not left in some stuck completed state.
    bool got_second = false;
    for (size_t i = 0; i < sizeof(kZeroFrame); i++) {
        got_second = crsf_frame_sync_push_byte(&sync, kZeroFrame[i], &frame_len);
    }
    CHECK(got_second, "a second frame immediately after the first completes normally");
    CHECK(memcmp(sync.buf, kZeroFrame, frame_len) == 0, "second frame's bytes are correct");
}

static void test_frame_sync_rejects_implausible_length_byte(void)
{
    crsf_frame_sync_t sync;
    crsf_frame_sync_reset(&sync);
    size_t frame_len;

    CHECK(!crsf_frame_sync_push_byte(&sync, CRSF_SYNC_BYTE, &frame_len), "sync byte alone");
    // A length byte of 0xFF would declare a frame far larger than CRSF_MAX_FRAME_SIZE -- must be
    // rejected (resync), not accepted and left to overflow the scratch buffer.
    CHECK(!crsf_frame_sync_push_byte(&sync, 0xFF, &frame_len),
          "an implausible length byte does not complete a frame");
    CHECK(sync.len == 0, "an implausible length byte resets sync state (not stuck accumulating)");
}

// --- channel mapping -------------------------------------------------------------------------

static crsf_channel_map_t default_map(void)
{
    crsf_channel_map_t map = {
        .throttle_channel = 0,
        .roll_channel = 1,
        .pitch_channel = 2,
        .yaw_channel = 3,
        .arm_channel = 4,
        .flight_mode_channel = 5,
        .raw_min = 172,
        .raw_center = 992,
        .raw_max = 1811,
        .arm_threshold = 992,
        .flight_mode_positions = 3,
    };
    return map;
}

static void test_channels_to_command_center_and_endpoints(void)
{
    crsf_channel_map_t map = default_map();
    crsf_rc_channels_t channels = { 0 };
    channels.raw[0] = 172;  // throttle at raw_min
    channels.raw[1] = 992;  // roll centered
    channels.raw[2] = 172;  // pitch at raw_min
    channels.raw[3] = 1811; // yaw at raw_max
    channels.raw[4] = 172;  // arm channel well below threshold
    channels.raw[5] = 992;  // flight mode channel mid-range

    crsf_command_t cmd;
    crsf_channels_to_command(&channels, &map, &cmd);

    CHECK_NEAR_F(cmd.throttle, 0.0f, 1e-4f, "throttle at raw_min maps to 0.0");
    CHECK_NEAR_F(cmd.roll, 0.0f, 1e-4f, "roll at raw_center maps to 0.0");
    CHECK_NEAR_F(cmd.pitch, -1.0f, 1e-4f, "pitch at raw_min maps to -1.0");
    CHECK_NEAR_F(cmd.yaw, 1.0f, 1e-4f, "yaw at raw_max maps to +1.0");
    CHECK(cmd.arm == false, "arm channel below threshold reads as disarmed");

    channels.raw[0] = 1811; // throttle at raw_max
    channels.raw[4] = 1811; // arm channel well above threshold
    crsf_channels_to_command(&channels, &map, &cmd);
    CHECK_NEAR_F(cmd.throttle, 1.0f, 1e-4f, "throttle at raw_max maps to 1.0");
    CHECK(cmd.arm == true, "arm channel at/above threshold reads as armed");
}

static void test_channels_to_command_clamps_out_of_range_raw_values(void)
{
    crsf_channel_map_t map = default_map();
    crsf_rc_channels_t channels = { 0 };
    channels.raw[0] = 0;    // below raw_min
    channels.raw[2] = 2047; // above raw_max, bipolar

    crsf_command_t cmd;
    crsf_channels_to_command(&channels, &map, &cmd);
    CHECK_NEAR_F(cmd.throttle, 0.0f, 1e-4f, "throttle below raw_min clamps to 0.0, not negative");
    CHECK_NEAR_F(cmd.pitch, 1.0f, 1e-4f, "pitch above raw_max clamps to +1.0, not beyond");
}

static void test_channels_to_command_flight_mode_quantization(void)
{
    crsf_channel_map_t map = default_map(); // flight_mode_positions = 3
    crsf_rc_channels_t channels = { 0 };

    channels.raw[5] = map.raw_min;
    crsf_command_t cmd;
    crsf_channels_to_command(&channels, &map, &cmd);
    CHECK(cmd.flight_mode == 0, "flight-mode channel at raw_min quantizes to bucket 0");

    channels.raw[5] = map.raw_max;
    crsf_channels_to_command(&channels, &map, &cmd);
    CHECK(cmd.flight_mode == 2, "flight-mode channel at raw_max quantizes to the last bucket (2)");

    channels.raw[5] = (uint16_t)((map.raw_min + map.raw_max) / 2);
    crsf_channels_to_command(&channels, &map, &cmd);
    CHECK(cmd.flight_mode == 1, "flight-mode channel at the midpoint quantizes to the middle bucket");
}

static void test_channels_to_command_out_of_range_channel_index_reads_zero(void)
{
    crsf_channel_map_t map = default_map();
    map.roll_channel = 200; // >= CRSF_NUM_CHANNELS -- a misconfigured Kconfig value
    crsf_rc_channels_t channels = { 0 };
    channels.raw[1] = 1811; // would be roll if the map weren't misconfigured

    crsf_command_t cmd;
    crsf_channels_to_command(&channels, &map, &cmd);
    // channel_raw() treats an out-of-range index as reading raw 0, not the 1811 that's actually
    // sitting in channels.raw[1] -- this map's roll axis is bipolar with raw_min=172 > 0, so raw 0
    // clamps to the -1.0 endpoint, not a "neutral" 0.0. The behavior under test is "never indexes
    // out of bounds," not "produces a neutral stick value."
    CHECK_NEAR_F(cmd.roll, -1.0f, 1e-4f,
                 "an out-of-range configured channel index reads as raw 0 (clamped to -1.0 on "
                 "this bipolar axis) instead of indexing OOB into channels.raw[1]");
}

// --- packet-loss estimate --------------------------------------------------------------------

static void test_loss_tracker_zero_before_two_frames(void)
{
    crsf_loss_tracker_t tracker;
    crsf_loss_tracker_reset(&tracker, 4000);
    CHECK(crsf_loss_tracker_loss_percent(&tracker) == 0.0f, "no frames recorded yet -> 0%%");

    crsf_loss_tracker_record(&tracker, 1000000);
    CHECK(crsf_loss_tracker_loss_percent(&tracker) == 0.0f,
          "a single recorded frame has no interval to estimate from yet -> 0%%");
}

static void test_loss_tracker_no_loss_at_nominal_rate(void)
{
    crsf_loss_tracker_t tracker;
    int64_t period = 4000;
    crsf_loss_tracker_reset(&tracker, period);

    int64_t now = 0;
    for (int i = 0; i < 100; i++) {
        crsf_loss_tracker_record(&tracker, now);
        now += period;
    }
    CHECK_NEAR_F(crsf_loss_tracker_loss_percent(&tracker), 0.0f, 1.0f,
                 "frames arriving exactly at the nominal period estimate ~0%% loss");
}

static void test_loss_tracker_detects_gaps(void)
{
    crsf_loss_tracker_t tracker;
    int64_t period = 4000;
    crsf_loss_tracker_reset(&tracker, period);

    // 10 frames received back-to-back, then a gap of 40 periods (39 missed frames) before one
    // more frame.
    int64_t now = 0;
    for (int i = 0; i < 10; i++) {
        crsf_loss_tracker_record(&tracker, now);
        now += period;
    }
    now += 40 * period;
    crsf_loss_tracker_record(&tracker, now);

    float loss = crsf_loss_tracker_loss_percent(&tracker);
    CHECK(loss > 50.0f, "a long gap between frames drives the loss estimate up substantially");
}

static void test_loss_tracker_disabled_with_nonpositive_period(void)
{
    crsf_loss_tracker_t tracker;
    crsf_loss_tracker_reset(&tracker, 0);
    crsf_loss_tracker_record(&tracker, 0);
    crsf_loss_tracker_record(&tracker, 1000000);
    CHECK(crsf_loss_tracker_loss_percent(&tracker) == 0.0f,
          "a non-positive nominal_frame_period_us disables the estimate (reports 0%%)");
}

int main(void)
{
    test_crc8_matches_published_check_value();
    test_crc8_empty_and_single_byte();

    test_parse_valid_mixed_frame();
    test_parse_zero_and_max_frames();
    test_parse_distinct_frame_catches_bit_offset_errors();
    test_parse_rejects_wrong_length();
    test_parse_rejects_bad_sync_byte();
    test_parse_rejects_length_byte_mismatch();
    test_parse_rejects_wrong_frame_type();
    test_parse_rejects_corrupted_payload();
    test_parse_rejects_corrupted_crc_byte();

    test_frame_sync_accumulates_one_frame();
    test_frame_sync_discards_garbage_before_sync_byte();
    test_frame_sync_resets_after_completing_a_frame();
    test_frame_sync_rejects_implausible_length_byte();

    test_channels_to_command_center_and_endpoints();
    test_channels_to_command_clamps_out_of_range_raw_values();
    test_channels_to_command_flight_mode_quantization();
    test_channels_to_command_out_of_range_channel_index_reads_zero();

    test_loss_tracker_zero_before_two_frames();
    test_loss_tracker_no_loss_at_nominal_rate();
    test_loss_tracker_detects_gaps();
    test_loss_tracker_disabled_with_nonpositive_period();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
