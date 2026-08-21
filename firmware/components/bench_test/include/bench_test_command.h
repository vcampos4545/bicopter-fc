// Pure, ESP-IDF-free line parser for the bench-test console (Milestone 18 - see
// docs/bench_test.md). No I/O, no hardware access, no global state - just turning one line of
// typed text into a typed command - so it is fully host-tested
// (tests/bench_test_command_test.c), the same split this project's other wire-format/command
// parsers use (radio_packet.c, crsf_frame.c, flight_mode.c - see AGENTS.md's driver-testing
// convention). firmware/components/bench_test/src/bench_test_console.c is the ESP-IDF-dependent
// half (UART I/O, line assembly) that calls this.
#ifndef BICOPTER_BENCH_TEST_COMMAND_H
#define BICOPTER_BENCH_TEST_COMMAND_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Longest line this parser accepts (including the terminating NUL). A longer line is rejected by
// the console layer (see bench_test_console.h) before it ever reaches this parser.
#define BENCH_TEST_COMMAND_MAX_LINE_LEN 96

// The exact, case-sensitive trailing token esc_test requires - the item 1 (docs/bench_test.md)
// "extra, deliberate confirmation... distinct from just the normal arm/throttle path" requirement.
// Deliberately not "y"/"yes"/a single character, so it can't be muscle-memory-typed by accident
// alongside a normal command.
#define BENCH_TEST_ESC_CONFIRM_TOKEN "CONFIRM"

typedef enum {
    BENCH_TEST_CMD_NONE = 0, // blank/whitespace-only line - nothing to do, not an error
    BENCH_TEST_CMD_HELP,
    BENCH_TEST_CMD_SENSOR_START, // begin continuous IMU/barometer print stream
    BENCH_TEST_CMD_SENSOR_STOP,
    BENCH_TEST_CMD_RADIO_START, // begin continuous RadioCommand print stream
    BENCH_TEST_CMD_RADIO_STOP,
    BENCH_TEST_CMD_SERVO,     // command a single servo to a specific angle, once
    BENCH_TEST_CMD_ESC_TEST,  // command a single motor to a low throttle, once - gated (see
                               // bench_test_command_t.confirmed and docs/bench_test.md)
} bench_test_command_type_t;

typedef enum {
    BENCH_TEST_PARSE_OK = 0,
    BENCH_TEST_PARSE_UNKNOWN_COMMAND,
    BENCH_TEST_PARSE_MISSING_ARGS,
    BENCH_TEST_PARSE_INVALID_ARG,
    BENCH_TEST_PARSE_LINE_TOO_LONG,
    // esc_test parsed every other argument successfully but the required trailing
    // BENCH_TEST_ESC_CONFIRM_TOKEN was absent or didn't match exactly - reported distinctly from
    // BENCH_TEST_PARSE_MISSING_ARGS so the console layer can print a targeted message instead of
    // a generic usage error (see docs/bench_test.md).
    BENCH_TEST_PARSE_MISSING_CONFIRMATION,
} bench_test_parse_result_t;

typedef struct {
    bench_test_command_type_t type;
    // Valid for BENCH_TEST_CMD_SERVO / BENCH_TEST_CMD_ESC_TEST: which of the two
    // ACTUATORS_NUM_UNITS tilt-rotor units (0 or 1) to command. Already range-checked to {0, 1} by
    // bench_test_parse_line() - a caller never needs to re-validate it.
    int unit;
    // BENCH_TEST_CMD_SERVO: commanded angle, in DEGREES (not radians - console input/output is
    // human-facing; the caller converts to radians before calling servo_output_write(), same
    // boundary-conversion rule AGENTS.md states for every other sensor/protocol-native unit).
    // BENCH_TEST_CMD_ESC_TEST: requested throttle, normalized [0.0, 1.0] as typed - NOT yet
    // clamped to CONFIG_BICOPTER_BENCH_TEST_ESC_MAX_THROTTLE_PERCENT; the caller (bench_test_task.c)
    // applies that safety ceiling, since it is a build-config value this ESP-IDF-free parser has
    // no access to.
    float value;
    // BENCH_TEST_CMD_ESC_TEST only: true iff the line carried the exact BENCH_TEST_ESC_CONFIRM_TOKEN
    // as its final token. bench_test_parse_line() never returns BENCH_TEST_CMD_ESC_TEST with this
    // false - an unconfirmed esc_test line parses as BENCH_TEST_PARSE_MISSING_CONFIRMATION instead
    // - so a caller can treat "type == BENCH_TEST_CMD_ESC_TEST" as "confirmed" by construction.
    // The field still exists (rather than being implicit) so that invariant is visible at the call
    // site and testable directly, not just assumed.
    bool confirmed;
} bench_test_command_t;

// Parses one line of console input into *out_cmd. `line` must be NUL-terminated; a trailing
// '\r'/'\n' and surrounding whitespace are stripped internally, so the console layer does not need
// to pre-trim. Recognized grammar (whitespace-separated tokens, command name case-sensitive
// lowercase):
//
//   help
//   sensor start | sensor stop
//   radio start | radio stop
//   servo <unit 0|1> <angle_deg>
//   esc_test <unit 0|1> <throttle 0.0-1.0> CONFIRM
//
// On success, returns BENCH_TEST_PARSE_OK and fills *out_cmd. On failure, returns a specific
// bench_test_parse_result_t and leaves *out_cmd zeroed (type == BENCH_TEST_CMD_NONE) - a caller
// must check the return value, not out_cmd->type, to distinguish "blank line" (BENCH_TEST_PARSE_OK,
// type NONE) from a rejected one.
bench_test_parse_result_t bench_test_parse_line(const char *line, bench_test_command_t *out_cmd);

// Human-readable command-list text (no trailing newline), printed by the "help" command and
// appended to any parse-error message. One place to edit so the parser's accepted grammar and the
// text describing it to an operator can't silently drift apart.
const char *bench_test_help_text(void);

// Human-readable name for a bench_test_parse_result_t, for logging/console error messages.
const char *bench_test_parse_result_name(bench_test_parse_result_t result);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_BENCH_TEST_COMMAND_H
