// Host test for the BENCH_TEST motor-disable gate (Milestone 18 - see docs/bench_test.md).
//
// pwm_esc_output.c (the ESP-IDF hardware-I/O half of the ESC driver) is not itself host-testable
// - it depends on driver/ledc.h and driver/gpio.h, same as every other hardware-I/O file in this
// project (see AGENTS.md's driver-testing convention). What *is* host-testable, and what this file
// checks, is the one thing pwm_esc_output.c's #if PWM_ESC_BENCH_TEST_MOTORS_DISABLED guards are
// actually built from: pwm_esc_bench_test.h's translation of
// CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED into PWM_ESC_BENCH_TEST_MOTORS_DISABLED.
//
// tests/CMakeLists.txt builds this same source file into TWO executables:
//   - pwm_esc_bench_test_gate_test (no extra compile definition) - simulates every normal build,
//     including every ESP-IDF build with the Kconfig option at its default "n": the macro must
//     resolve to 0.
//   - pwm_esc_bench_test_gate_disabled_test (compiled with
//     -DCONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=1) - simulates a real BENCH_TEST build:
//     the macro must resolve to 1.
//
// Together these two prove the header's translation is correct under both configurations the real
// firmware build can produce, which is what pwm_esc_output.c's #if actually branches on. What this
// does NOT prove (and docs/bench_test.md says so explicitly, rather than overclaiming): that
// ledc_set_duty_and_update() is unreachable in the compiled BENCH_TEST *firmware* binary - that
// part rests on pwm_esc_output.c review (it is the only call site, and the whole function body is
// wrapped in the same #if this test exercises), not on a host test, since the ESP-IDF-dependent
// file itself cannot be compiled here.
#include <stdio.h>

#include "pwm_esc_bench_test.h"

static int g_failures = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);                                \
            g_failures++;                                                                         \
        } else {                                                                                  \
            printf("PASS: %s\n", msg);                                                            \
        }                                                                                          \
    } while (0)

int main(void)
{
#ifdef PWM_ESC_BENCH_TEST_GATE_EXPECT_DISABLED
    CHECK(PWM_ESC_BENCH_TEST_MOTORS_DISABLED == 1,
          "PWM_ESC_BENCH_TEST_MOTORS_DISABLED is 1 when "
          "CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED=1 (a real BENCH_TEST build)");
#else
    CHECK(PWM_ESC_BENCH_TEST_MOTORS_DISABLED == 0,
          "PWM_ESC_BENCH_TEST_MOTORS_DISABLED is 0 when CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED "
          "is undefined (every non-BENCH_TEST build, including the Kconfig option's own default)");
#endif

    if (g_failures == 0) {
        printf("All checks passed.\n");
        return 0;
    }
    printf("%d check(s) failed.\n", g_failures);
    return 1;
}
