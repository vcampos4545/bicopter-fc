#include "bench_test_command.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TOKENS 4

static const char *HELP_TEXT =
    "bench-test console commands (see docs/bench_test.md):\n"
    "  help                              - show this text\n"
    "  sensor start | sensor stop        - print IMU/barometer readings continuously\n"
    "  radio start  | radio stop         - print received RadioCommand values continuously\n"
    "  servo <unit 0|1> <angle_deg>      - command one tilt servo directly, once\n"
    "  esc_test <unit 0|1> <throttle 0-1> CONFIRM\n"
    "                                    - command one motor to a low throttle, once\n"
    "                                      (HARDWARE_TEST build only; requires the literal\n"
    "                                      trailing CONFIRM token)";

const char *bench_test_help_text(void)
{
    return HELP_TEXT;
}

const char *bench_test_parse_result_name(bench_test_parse_result_t result)
{
    switch (result) {
    case BENCH_TEST_PARSE_OK:
        return "OK";
    case BENCH_TEST_PARSE_UNKNOWN_COMMAND:
        return "UNKNOWN_COMMAND";
    case BENCH_TEST_PARSE_MISSING_ARGS:
        return "MISSING_ARGS";
    case BENCH_TEST_PARSE_INVALID_ARG:
        return "INVALID_ARG";
    case BENCH_TEST_PARSE_LINE_TOO_LONG:
        return "LINE_TOO_LONG";
    case BENCH_TEST_PARSE_MISSING_CONFIRMATION:
        return "MISSING_CONFIRMATION";
    default:
        return "UNKNOWN";
    }
}

// Splits `buf` (already NUL-terminated and mutable) in place on ASCII whitespace, filling
// `tokens[]` with pointers into `buf` and returning the token count (capped at MAX_TOKENS - a
// line with more tokens than that is reported by the caller as too many args via a normal
// mismatch path below, not a buffer overrun here).
static int tokenize(char *buf, char *tokens[MAX_TOKENS])
{
    int count = 0;
    char *saveptr = NULL;
    char *tok = strtok_r(buf, " \t", &saveptr);
    while (tok != NULL && count < MAX_TOKENS) {
        tokens[count++] = tok;
        tok = strtok_r(NULL, " \t", &saveptr);
    }
    return count;
}

// Parses a base-10 integer unit index token, accepting only exactly "0" or "1" (this project's
// fixed ACTUATORS_NUM_UNITS topology - see actuators_init.h). Anything else (out-of-range,
// trailing garbage, non-numeric) is rejected rather than silently clamped, per AGENTS.md's "never
// accept an out-of-range value silently" rule this project applies everywhere else.
static bool parse_unit(const char *token, int *out_unit)
{
    if (token[0] != '\0' && (token[0] == '0' || token[0] == '1') && token[1] == '\0') {
        *out_unit = token[0] - '0';
        return true;
    }
    return false;
}

// Parses a float token strictly: the whole token must be consumed (no trailing garbage like
// "1.5abc"), and the result must be finite - same discipline pwm_esc_convert.c/servo_convert.c
// apply to every other user-facing numeric input in this project.
static bool parse_float_strict(const char *token, float *out_value)
{
    if (token[0] == '\0') {
        return false;
    }
    char *end = NULL;
    double v = strtod(token, &end);
    if (end == token || *end != '\0') {
        return false;
    }
    if (v != v || v > 3.4e38 || v < -3.4e38) { // NaN or out of float range
        return false;
    }
    *out_value = (float)v;
    return true;
}

bench_test_parse_result_t bench_test_parse_line(const char *line, bench_test_command_t *out_cmd)
{
    bench_test_command_t cmd = {0};
    if (out_cmd != NULL) {
        *out_cmd = cmd;
    }
    if (line == NULL || out_cmd == NULL) {
        return BENCH_TEST_PARSE_INVALID_ARG;
    }

    size_t len = strlen(line);
    // Strip a trailing '\r' and/or '\n' before the length check, so a line at exactly the limit
    // followed by the console layer's own newline isn't rejected for a byte it doesn't actually
    // keep.
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == '\n')) {
        len--;
    }
    if (len >= BENCH_TEST_COMMAND_MAX_LINE_LEN) {
        return BENCH_TEST_PARSE_LINE_TOO_LONG;
    }

    char buf[BENCH_TEST_COMMAND_MAX_LINE_LEN];
    memcpy(buf, line, len);
    buf[len] = '\0';

    char *tokens[MAX_TOKENS];
    int n = tokenize(buf, tokens);

    if (n == 0) {
        *out_cmd = cmd; // type == BENCH_TEST_CMD_NONE
        return BENCH_TEST_PARSE_OK;
    }

    if (strcmp(tokens[0], "help") == 0) {
        if (n != 1) {
            return BENCH_TEST_PARSE_MISSING_ARGS;
        }
        cmd.type = BENCH_TEST_CMD_HELP;
        *out_cmd = cmd;
        return BENCH_TEST_PARSE_OK;
    }

    if (strcmp(tokens[0], "sensor") == 0) {
        if (n != 2) {
            return BENCH_TEST_PARSE_MISSING_ARGS;
        }
        if (strcmp(tokens[1], "start") == 0) {
            cmd.type = BENCH_TEST_CMD_SENSOR_START;
        } else if (strcmp(tokens[1], "stop") == 0) {
            cmd.type = BENCH_TEST_CMD_SENSOR_STOP;
        } else {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        *out_cmd = cmd;
        return BENCH_TEST_PARSE_OK;
    }

    if (strcmp(tokens[0], "radio") == 0) {
        if (n != 2) {
            return BENCH_TEST_PARSE_MISSING_ARGS;
        }
        if (strcmp(tokens[1], "start") == 0) {
            cmd.type = BENCH_TEST_CMD_RADIO_START;
        } else if (strcmp(tokens[1], "stop") == 0) {
            cmd.type = BENCH_TEST_CMD_RADIO_STOP;
        } else {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        *out_cmd = cmd;
        return BENCH_TEST_PARSE_OK;
    }

    if (strcmp(tokens[0], "servo") == 0) {
        if (n < 3) {
            return BENCH_TEST_PARSE_MISSING_ARGS;
        }
        if (n > 3) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        int unit;
        float angle_deg;
        if (!parse_unit(tokens[1], &unit)) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        if (!parse_float_strict(tokens[2], &angle_deg)) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        cmd.type = BENCH_TEST_CMD_SERVO;
        cmd.unit = unit;
        cmd.value = angle_deg;
        *out_cmd = cmd;
        return BENCH_TEST_PARSE_OK;
    }

    if (strcmp(tokens[0], "esc_test") == 0) {
        if (n < 3) {
            return BENCH_TEST_PARSE_MISSING_ARGS;
        }
        if (n > 4) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        int unit;
        float throttle;
        if (!parse_unit(tokens[1], &unit)) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        if (!parse_float_strict(tokens[2], &throttle)) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        if (throttle < 0.0f || throttle > 1.0f) {
            return BENCH_TEST_PARSE_INVALID_ARG;
        }
        // The confirmation token is mandatory and exact-match, checked last so a genuinely
        // malformed command (bad unit/throttle) is reported as such rather than masked by a
        // missing-confirmation message.
        if (n != 4 || strcmp(tokens[3], BENCH_TEST_ESC_CONFIRM_TOKEN) != 0) {
            return BENCH_TEST_PARSE_MISSING_CONFIRMATION;
        }
        cmd.type = BENCH_TEST_CMD_ESC_TEST;
        cmd.unit = unit;
        cmd.value = throttle;
        cmd.confirmed = true;
        *out_cmd = cmd;
        return BENCH_TEST_PARSE_OK;
    }

    return BENCH_TEST_PARSE_UNKNOWN_COMMAND;
}
