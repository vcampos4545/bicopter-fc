// Real automated tests for the bench-test console's pure line parser (bench_test_command.c) -
// Milestone 18, see docs/bench_test.md. Same hand-rolled harness as
// tests/radio_packet_test.c/tests/mpu6050_convert_test.c: a host-side CTest binary compiled
// independent of ESP-IDF, no test framework dependency needed.
// firmware/components/bench_test/src/bench_test_console.c (the UART driver I/O half) is
// ESP-IDF-dependent and not host-testable - see AGENTS.md's driver-testing convention.

#include <string.h>
#include <stdio.h>

#include "bench_test_command.h"

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                          \
    do {                                                                                           \
        g_checks++;                                                                                \
        if (!(cond)) {                                                                             \
            g_failures++;                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                                   \
        }                                                                                          \
    } while (0)

static void test_blank_line_is_none(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("", &cmd) == BENCH_TEST_PARSE_OK, "empty line parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_NONE, "empty line -> CMD_NONE");

    CHECK(bench_test_parse_line("   \t  ", &cmd) == BENCH_TEST_PARSE_OK,
          "whitespace-only line parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_NONE, "whitespace-only line -> CMD_NONE");
}

static void test_help(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("help", &cmd) == BENCH_TEST_PARSE_OK, "'help' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_HELP, "'help' -> CMD_HELP");

    CHECK(bench_test_parse_line("help extra", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'help extra' rejected (too many args)");
}

static void test_sensor_start_stop(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("sensor start", &cmd) == BENCH_TEST_PARSE_OK,
          "'sensor start' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_SENSOR_START, "'sensor start' -> CMD_SENSOR_START");

    CHECK(bench_test_parse_line("sensor stop", &cmd) == BENCH_TEST_PARSE_OK,
          "'sensor stop' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_SENSOR_STOP, "'sensor stop' -> CMD_SENSOR_STOP");

    CHECK(bench_test_parse_line("sensor", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'sensor' alone rejected (missing start/stop)");
    CHECK(bench_test_parse_line("sensor toggle", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'sensor toggle' rejected (unrecognized sub-arg)");
}

static void test_radio_start_stop(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("radio start", &cmd) == BENCH_TEST_PARSE_OK,
          "'radio start' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_RADIO_START, "'radio start' -> CMD_RADIO_START");

    CHECK(bench_test_parse_line("radio stop", &cmd) == BENCH_TEST_PARSE_OK,
          "'radio stop' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_RADIO_STOP, "'radio stop' -> CMD_RADIO_STOP");
}

static void test_servo(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("servo 0 15.5", &cmd) == BENCH_TEST_PARSE_OK,
          "'servo 0 15.5' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_SERVO, "'servo 0 15.5' -> CMD_SERVO");
    CHECK(cmd.unit == 0, "'servo 0 15.5' unit == 0");
    CHECK(cmd.value > 15.49f && cmd.value < 15.51f, "'servo 0 15.5' value ~= 15.5");

    CHECK(bench_test_parse_line("servo 1 -30", &cmd) == BENCH_TEST_PARSE_OK,
          "'servo 1 -30' parses OK");
    CHECK(cmd.unit == 1, "'servo 1 -30' unit == 1");
    CHECK(cmd.value > -30.01f && cmd.value < -29.99f, "'servo 1 -30' value ~= -30");

    CHECK(bench_test_parse_line("servo 2 0", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'servo 2 0' rejected (unit out of {0,1})");
    CHECK(bench_test_parse_line("servo -1 0", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'servo -1 0' rejected (unit out of {0,1})");
    CHECK(bench_test_parse_line("servo 0 abc", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'servo 0 abc' rejected (non-numeric angle)");
    CHECK(bench_test_parse_line("servo 0 1.5abc", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'servo 0 1.5abc' rejected (trailing garbage on angle)");
    CHECK(bench_test_parse_line("servo 0", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'servo 0' rejected (missing angle)");
    CHECK(bench_test_parse_line("servo", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'servo' rejected (missing all args)");
    CHECK(bench_test_parse_line("servo 0 15 extra", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'servo 0 15 extra' rejected (too many args)");
}

static void test_esc_test_requires_exact_confirm_token(void)
{
    bench_test_command_t cmd;

    CHECK(bench_test_parse_line("esc_test 0 0.1 CONFIRM", &cmd) == BENCH_TEST_PARSE_OK,
          "'esc_test 0 0.1 CONFIRM' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_ESC_TEST, "'esc_test 0 0.1 CONFIRM' -> CMD_ESC_TEST");
    CHECK(cmd.unit == 0, "'esc_test 0 0.1 CONFIRM' unit == 0");
    CHECK(cmd.value > 0.099f && cmd.value < 0.101f, "'esc_test 0 0.1 CONFIRM' value ~= 0.1");
    CHECK(cmd.confirmed == true, "'esc_test 0 0.1 CONFIRM' confirmed == true");

    // Every one of these is a plausible near-miss an operator could actually type - the whole
    // point of item 1's "extra, deliberate confirmation" requirement (docs/bench_test.md) is that
    // none of them succeed.
    CHECK(bench_test_parse_line("esc_test 0 0.1", &cmd) == BENCH_TEST_PARSE_MISSING_CONFIRMATION,
          "'esc_test 0 0.1' (no confirm token) rejected as MISSING_CONFIRMATION");
    CHECK(bench_test_parse_line("esc_test 0 0.1 confirm", &cmd) ==
              BENCH_TEST_PARSE_MISSING_CONFIRMATION,
          "'esc_test 0 0.1 confirm' (wrong case) rejected as MISSING_CONFIRMATION");
    CHECK(bench_test_parse_line("esc_test 0 0.1 yes", &cmd) == BENCH_TEST_PARSE_MISSING_CONFIRMATION,
          "'esc_test 0 0.1 yes' rejected as MISSING_CONFIRMATION");
    CHECK(bench_test_parse_line("esc_test 0 0.1 CONFIRMED", &cmd) ==
              BENCH_TEST_PARSE_MISSING_CONFIRMATION,
          "'esc_test 0 0.1 CONFIRMED' (near-miss token) rejected as MISSING_CONFIRMATION");
    CHECK(bench_test_parse_line("esc_test 0 0.1 CONFIR", &cmd) ==
              BENCH_TEST_PARSE_MISSING_CONFIRMATION,
          "'esc_test 0 0.1 CONFIR' (truncated token) rejected as MISSING_CONFIRMATION");

    // A blank line never accidentally re-triggers the last esc_test, and a rejected line never
    // populates a command a caller might mistakenly act on.
    CHECK(bench_test_parse_line("esc_test 0 0.1", &cmd) != BENCH_TEST_PARSE_OK ||
              cmd.type != BENCH_TEST_CMD_ESC_TEST,
          "an unconfirmed esc_test line never yields CMD_ESC_TEST");
}

static void test_esc_test_validates_unit_and_throttle_before_confirmation(void)
{
    bench_test_command_t cmd;

    // A malformed unit/throttle is reported as such, not masked by a missing-confirmation message
    // - see bench_test_parse_line()'s header comment on check ordering.
    CHECK(bench_test_parse_line("esc_test 5 0.1 CONFIRM", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'esc_test 5 0.1 CONFIRM' rejected as INVALID_ARG (bad unit), not MISSING_CONFIRMATION");
    CHECK(bench_test_parse_line("esc_test 0 1.5 CONFIRM", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'esc_test 0 1.5 CONFIRM' rejected as INVALID_ARG (throttle > 1.0)");
    CHECK(bench_test_parse_line("esc_test 0 -0.1 CONFIRM", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'esc_test 0 -0.1 CONFIRM' rejected as INVALID_ARG (throttle < 0.0)");
    CHECK(bench_test_parse_line("esc_test 0 nan CONFIRM", &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "'esc_test 0 nan CONFIRM' rejected as INVALID_ARG");

    CHECK(bench_test_parse_line("esc_test 0 1.0 CONFIRM", &cmd) == BENCH_TEST_PARSE_OK,
          "'esc_test 0 1.0 CONFIRM' (throttle at the upper boundary) parses OK");
    CHECK(bench_test_parse_line("esc_test 0 0.0 CONFIRM", &cmd) == BENCH_TEST_PARSE_OK,
          "'esc_test 0 0.0 CONFIRM' (throttle at the lower boundary) parses OK");

    CHECK(bench_test_parse_line("esc_test 0", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'esc_test 0' rejected as MISSING_ARGS (no throttle, no confirm)");
    CHECK(bench_test_parse_line("esc_test", &cmd) == BENCH_TEST_PARSE_MISSING_ARGS,
          "'esc_test' rejected as MISSING_ARGS");
}

static void test_unknown_command(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("takeoff", &cmd) == BENCH_TEST_PARSE_UNKNOWN_COMMAND,
          "'takeoff' rejected as UNKNOWN_COMMAND");
    CHECK(bench_test_parse_line("ESC_TEST 0 0.1 CONFIRM", &cmd) ==
              BENCH_TEST_PARSE_UNKNOWN_COMMAND,
          "command name is case-sensitive: 'ESC_TEST' is UNKNOWN_COMMAND, not 'esc_test'");
}

static void test_trailing_crlf_and_whitespace_stripped(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line("help\r\n", &cmd) == BENCH_TEST_PARSE_OK,
          "trailing CRLF stripped: 'help\\r\\n' parses OK");
    CHECK(cmd.type == BENCH_TEST_CMD_HELP, "trailing CRLF stripped: -> CMD_HELP");

    CHECK(bench_test_parse_line("  sensor start  ", &cmd) == BENCH_TEST_PARSE_OK,
          "leading/trailing whitespace tolerated");
    CHECK(cmd.type == BENCH_TEST_CMD_SENSOR_START, "leading/trailing whitespace -> CMD_SENSOR_START");
}

static void test_line_too_long_rejected(void)
{
    char line[BENCH_TEST_COMMAND_MAX_LINE_LEN + 32];
    memset(line, 'a', sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';

    bench_test_command_t cmd;
    CHECK(bench_test_parse_line(line, &cmd) == BENCH_TEST_PARSE_LINE_TOO_LONG,
          "an over-length line is rejected as LINE_TOO_LONG, not truncated and parsed");
}

static void test_null_args_rejected(void)
{
    bench_test_command_t cmd;
    CHECK(bench_test_parse_line(NULL, &cmd) == BENCH_TEST_PARSE_INVALID_ARG,
          "NULL line rejected");
    CHECK(bench_test_parse_line("help", NULL) == BENCH_TEST_PARSE_INVALID_ARG,
          "NULL out_cmd rejected");
}

static void test_help_text_and_result_names_nonempty(void)
{
    CHECK(bench_test_help_text() != NULL && strlen(bench_test_help_text()) > 0,
          "bench_test_help_text() is non-empty");
    for (int r = BENCH_TEST_PARSE_OK; r <= BENCH_TEST_PARSE_MISSING_CONFIRMATION; r++) {
        const char *name = bench_test_parse_result_name((bench_test_parse_result_t)r);
        CHECK(name != NULL && strlen(name) > 0, "bench_test_parse_result_name() is non-empty");
    }
}

int main(void)
{
    test_blank_line_is_none();
    test_help();
    test_sensor_start_stop();
    test_radio_start_stop();
    test_servo();
    test_esc_test_requires_exact_confirm_token();
    test_esc_test_validates_unit_and_throttle_before_confirmation();
    test_unknown_command();
    test_trailing_crlf_and_whitespace_stripped();
    test_line_too_long_rejected();
    test_null_args_rejected();
    test_help_text_and_result_names_nonempty();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
