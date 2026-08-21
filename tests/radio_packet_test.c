// Real automated tests for the radio component's pure packet logic (radio_packet.c): command/
// telemetry packet serialize/parse round-trips, malformed/undersized packet rejection,
// sequence-number staleness/reordering detection (with wraparound), and packet-loss-percentage
// tracking. Same hand-rolled harness as tests/mpu6050_convert_test.c and tests/pwm_util_test.c
// (see those files for why: this is a host-side CTest binary compiled independent of ESP-IDF, no
// test framework dependency needed). Everything in firmware/components/radio/src/esp_now_radio.c
// (the actual ESP-NOW Wi-Fi calls) is ESP-IDF/hardware-dependent and not host-testable -- see
// docs/radio.md's verified-vs-deferred section for the honest line between what this file covers
// and what remains reviewed-but-unexercised.

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "radio_packet.h"

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

// --- command packet serialize/parse ------------------------------------------------------------

static void test_command_round_trip(void)
{
    radio_command_packet_t packet = {
        .sequence = 42,
        .timestamp_us = 1234567890123LL,
        .throttle = 0.75f,
        .roll = -0.5f,
        .pitch = 0.25f,
        .yaw = -1.0f,
        .arm = true,
        .flight_mode = 3,
    };

    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    size_t n = radio_command_packet_serialize(&packet, buf, sizeof(buf));
    CHECK(n == RADIO_COMMAND_PACKET_WIRE_SIZE, "serialize returns the full wire size");

    radio_command_packet_t parsed;
    bool ok = radio_command_packet_parse(buf, n, &parsed);
    CHECK(ok, "a freshly serialized command packet parses successfully");
    CHECK(parsed.sequence == packet.sequence, "sequence round-trips");
    CHECK(parsed.timestamp_us == packet.timestamp_us, "timestamp_us round-trips");
    CHECK_NEAR_F(parsed.throttle, packet.throttle, 1e-6f, "throttle round-trips");
    CHECK_NEAR_F(parsed.roll, packet.roll, 1e-6f, "roll round-trips");
    CHECK_NEAR_F(parsed.pitch, packet.pitch, 1e-6f, "pitch round-trips");
    CHECK_NEAR_F(parsed.yaw, packet.yaw, 1e-6f, "yaw round-trips");
    CHECK(parsed.arm == packet.arm, "arm round-trips");
    CHECK(parsed.flight_mode == packet.flight_mode, "flight_mode round-trips");
}

static void test_command_arm_false_round_trips(void)
{
    radio_command_packet_t packet = { .arm = false, .flight_mode = 0 };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));

    radio_command_packet_t parsed;
    CHECK(radio_command_packet_parse(buf, sizeof(buf), &parsed), "disarmed packet parses");
    CHECK(parsed.arm == false, "arm=false round-trips as false, not just 'nonzero'");
}

static void test_command_serialize_rejects_undersized_buffer(void)
{
    radio_command_packet_t packet = { 0 };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE - 1];
    size_t n = radio_command_packet_serialize(&packet, buf, sizeof(buf));
    CHECK(n == 0, "serialize into a too-small buffer returns 0, writes nothing usable");
}

static void test_command_parse_rejects_wrong_length(void)
{
    radio_command_packet_t packet = { .throttle = 0.5f };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));

    radio_command_packet_t parsed;
    CHECK(!radio_command_packet_parse(buf, sizeof(buf) - 1, &parsed),
          "one byte short of the wire size is rejected");
    CHECK(!radio_command_packet_parse(buf, sizeof(buf) + 1, &parsed),
          "one byte too long is rejected");
    CHECK(!radio_command_packet_parse(buf, 0, &parsed), "zero-length packet is rejected");
}

static void test_command_parse_rejects_bad_magic(void)
{
    radio_command_packet_t packet = { .throttle = 0.5f };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));
    buf[0] ^= 0xFF; // corrupt the magic byte

    radio_command_packet_t parsed;
    CHECK(!radio_command_packet_parse(buf, sizeof(buf), &parsed),
          "a foreign/non-bicopter packet (wrong magic byte) is rejected");
}

static void test_command_parse_rejects_bad_version(void)
{
    radio_command_packet_t packet = { .throttle = 0.5f };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));
    buf[1] = (uint8_t)(RADIO_PACKET_VERSION + 1);
    // checksum now stale -- recompute so this test isolates the version check, not the checksum
    // check.
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(buf) - 1; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[sizeof(buf) - 1] = sum;

    radio_command_packet_t parsed;
    CHECK(!radio_command_packet_parse(buf, sizeof(buf), &parsed),
          "a mismatched protocol version is rejected");
}

static void test_command_parse_rejects_corrupted_checksum(void)
{
    radio_command_packet_t packet = { .throttle = 0.5f };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));
    buf[10] ^= 0x01; // flip one payload bit without touching the checksum byte

    radio_command_packet_t parsed;
    CHECK(!radio_command_packet_parse(buf, sizeof(buf), &parsed),
          "a single-bit payload corruption is caught by the checksum");
}

static void test_command_parse_rejects_non_finite_float(void)
{
    radio_command_packet_t packet = { .throttle = 0.5f };
    uint8_t buf[RADIO_COMMAND_PACKET_WIRE_SIZE];
    radio_command_packet_serialize(&packet, buf, sizeof(buf));

    // Overwrite the throttle field (bytes 6-9: magic, version, 4 seq, 8 ts, then throttle) with a
    // NaN bit pattern and fix up the checksum, so this isolates the finite-value check from the
    // checksum check.
    size_t throttle_offset = 1 + 1 + 4 + 8;
    uint32_t nan_bits = 0x7FC00000u; // a quiet NaN
    memcpy(&buf[throttle_offset], &nan_bits, sizeof(nan_bits));
    uint8_t sum = 0;
    for (size_t i = 0; i < sizeof(buf) - 1; i++) {
        sum = (uint8_t)(sum + buf[i]);
    }
    buf[sizeof(buf) - 1] = sum;

    radio_command_packet_t parsed;
    CHECK(!radio_command_packet_parse(buf, sizeof(buf), &parsed),
          "a NaN field is rejected even with a matching checksum/magic/version");
}

// --- telemetry packet serialize/parse ----------------------------------------------------------

static void test_telemetry_round_trip(void)
{
    radio_telemetry_packet_t packet = {
        .sequence = 7,
        .timestamp_us = 999,
        .armed = true,
        .flight_mode = 1,
        .packet_loss_percent = 12.5f,
    };
    uint8_t buf[RADIO_TELEMETRY_PACKET_WIRE_SIZE];
    size_t n = radio_telemetry_packet_serialize(&packet, buf, sizeof(buf));
    CHECK(n == RADIO_TELEMETRY_PACKET_WIRE_SIZE, "telemetry serialize returns the full wire size");

    radio_telemetry_packet_t parsed;
    CHECK(radio_telemetry_packet_parse(buf, n, &parsed), "telemetry packet parses successfully");
    CHECK(parsed.sequence == packet.sequence, "telemetry sequence round-trips");
    CHECK(parsed.armed == packet.armed, "telemetry armed round-trips");
    CHECK(parsed.flight_mode == packet.flight_mode, "telemetry flight_mode round-trips");
    CHECK_NEAR_F(parsed.packet_loss_percent, packet.packet_loss_percent, 1e-4f,
                 "telemetry packet_loss_percent round-trips");
}

static void test_telemetry_parse_rejects_wrong_length(void)
{
    uint8_t buf[RADIO_TELEMETRY_PACKET_WIRE_SIZE] = { 0 };
    radio_telemetry_packet_t parsed;
    CHECK(!radio_telemetry_packet_parse(buf, sizeof(buf) - 1, &parsed),
          "telemetry packet one byte short is rejected");
}

// --- sequence-number staleness/reordering --------------------------------------------------

static void test_seq_first_packet_always_accepted(void)
{
    CHECK(radio_seq_is_newer(0, 0, false), "any sequence is accepted with no prior sequence");
    CHECK(radio_seq_is_newer(12345, 0, false),
          "a nonzero first sequence is accepted with no prior sequence");
}

static void test_seq_strictly_increasing_accepted(void)
{
    CHECK(radio_seq_is_newer(5, 4, true), "seq 5 after seq 4 is accepted");
    CHECK(radio_seq_is_newer(100, 4, true), "a jump forward is accepted (gap = loss, not reject)");
}

static void test_seq_duplicate_rejected(void)
{
    CHECK(!radio_seq_is_newer(4, 4, true), "a duplicate/replayed sequence is rejected");
}

static void test_seq_reordered_rejected(void)
{
    CHECK(!radio_seq_is_newer(3, 4, true), "an out-of-order (older) sequence is rejected");
    CHECK(!radio_seq_is_newer(0, 4, true), "an out-of-order sequence of 0 is rejected");
}

static void test_seq_wraparound_accepted(void)
{
    uint32_t last = 0xFFFFFFFFu;
    CHECK(radio_seq_is_newer(0, last, true),
          "sequence wraparound (0 after UINT32_MAX) is treated as newer, not older");
    CHECK(radio_seq_is_newer(5, last, true), "a few sequences past wraparound are still newer");
}

static void test_seq_far_future_not_confused_with_wraparound(void)
{
    // A sequence more than 2^31 ahead is, by RFC1982 serial-number-arithmetic convention,
    // ambiguous/treated as "older" -- this can't happen in practice (would need ~2 billion
    // dropped packets) but documents the boundary this implementation chose rather than leaving
    // it accidental.
    uint32_t last = 0;
    uint32_t far_future = 0x80000000u;
    CHECK(!radio_seq_is_newer(far_future, last, true),
          "exactly halfway around the sequence space is not treated as newer");
}

// --- packet-loss tracking ------------------------------------------------------------------

static void test_loss_tracker_no_data_is_zero_percent(void)
{
    radio_loss_tracker_t tracker;
    radio_loss_tracker_reset(&tracker);
    CHECK_NEAR_F(radio_loss_tracker_loss_percent(&tracker), 0.0f, 1e-6f,
                 "an untouched tracker reports 0% loss, not a divide-by-zero result");
}

static void test_loss_tracker_no_gaps_is_zero_percent(void)
{
    radio_loss_tracker_t tracker;
    radio_loss_tracker_reset(&tracker);
    for (uint32_t seq = 0; seq < 10; seq++) {
        radio_loss_tracker_record(&tracker, seq);
    }
    CHECK_NEAR_F(radio_loss_tracker_loss_percent(&tracker), 0.0f, 1e-4f,
                 "ten consecutive sequences with no gaps report 0% loss");
}

static void test_loss_tracker_counts_gaps(void)
{
    radio_loss_tracker_t tracker;
    radio_loss_tracker_reset(&tracker);
    radio_loss_tracker_record(&tracker, 0);
    radio_loss_tracker_record(&tracker, 1);
    radio_loss_tracker_record(&tracker, 4); // 2 dropped (seq 2, 3) before this one
    // received=3, expected=1(first)+1+3=5 -> loss = 100*(1-3/5) = 40%
    CHECK_NEAR_F(radio_loss_tracker_loss_percent(&tracker), 40.0f, 1e-3f,
                 "a known gap pattern produces the hand-computed loss percentage");
}

static void test_loss_tracker_reset_clears_state(void)
{
    radio_loss_tracker_t tracker;
    radio_loss_tracker_reset(&tracker);
    radio_loss_tracker_record(&tracker, 0);
    radio_loss_tracker_record(&tracker, 50); // big gap
    CHECK(radio_loss_tracker_loss_percent(&tracker) > 0.0f, "sanity: loss is nonzero before reset");

    radio_loss_tracker_reset(&tracker);
    CHECK(!tracker.has_last_sequence, "reset clears has_last_sequence");
    CHECK_NEAR_F(radio_loss_tracker_loss_percent(&tracker), 0.0f, 1e-6f,
                 "reset clears loss back to 0%");
}

// --- staleness ------------------------------------------------------------------------------

static void test_is_stale_within_timeout_is_fresh(void)
{
    CHECK(!radio_is_stale(1000, 1400, 500), "400us since last valid rx, 500us timeout -> not stale");
}

static void test_is_stale_exactly_at_timeout_is_fresh(void)
{
    CHECK(!radio_is_stale(1000, 1500, 500),
          "exactly at the timeout boundary is not yet stale (strictly-greater-than semantics)");
}

static void test_is_stale_past_timeout_is_stale(void)
{
    CHECK(radio_is_stale(1000, 1501, 500), "one microsecond past the timeout is stale");
    CHECK(radio_is_stale(1000, 100000, 500), "far past the timeout is stale");
}

int main(void)
{
    test_command_round_trip();
    test_command_arm_false_round_trips();
    test_command_serialize_rejects_undersized_buffer();
    test_command_parse_rejects_wrong_length();
    test_command_parse_rejects_bad_magic();
    test_command_parse_rejects_bad_version();
    test_command_parse_rejects_corrupted_checksum();
    test_command_parse_rejects_non_finite_float();

    test_telemetry_round_trip();
    test_telemetry_parse_rejects_wrong_length();

    test_seq_first_packet_always_accepted();
    test_seq_strictly_increasing_accepted();
    test_seq_duplicate_rejected();
    test_seq_reordered_rejected();
    test_seq_wraparound_accepted();
    test_seq_far_future_not_confused_with_wraparound();

    test_loss_tracker_no_data_is_zero_percent();
    test_loss_tracker_no_gaps_is_zero_percent();
    test_loss_tracker_counts_gaps();
    test_loss_tracker_reset_clears_state();

    test_is_stale_within_timeout_is_fresh();
    test_is_stale_exactly_at_timeout_is_fresh();
    test_is_stale_past_timeout_is_stale();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
