// Real automated test for the one piece of Milestone 6's task logic pure enough to host-test:
// SensorTask's baro-read decimation check (firmware/main/task_util.c). Same hand-rolled harness
// as tests/pwm_util_test.c (see that file for why).

#include <stdio.h>

#include "task_util.h"

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

static void test_decimation_of_zero_or_one_is_every_cycle(void)
{
    CHECK(task_should_decimate(1, 0), "decimation=0 is every cycle (no divide-by-zero)");
    CHECK(task_should_decimate(2, 0), "decimation=0 is every cycle (no divide-by-zero)");
    CHECK(task_should_decimate(1, 1), "decimation=1 is every cycle");
    CHECK(task_should_decimate(7, 1), "decimation=1 is every cycle");
}

static void test_decimation_of_ten_fires_every_tenth_cycle(void)
{
    CHECK(task_should_decimate(10, 10), "cycle 10 fires at decimation 10");
    CHECK(task_should_decimate(20, 10), "cycle 20 fires at decimation 10");
    CHECK(task_should_decimate(100, 10), "cycle 100 fires at decimation 10");
}

static void test_decimation_of_ten_skips_intermediate_cycles(void)
{
    CHECK(!task_should_decimate(1, 10), "cycle 1 does not fire at decimation 10");
    CHECK(!task_should_decimate(9, 10), "cycle 9 does not fire at decimation 10");
    CHECK(!task_should_decimate(11, 10), "cycle 11 does not fire at decimation 10");
    CHECK(!task_should_decimate(99, 10), "cycle 99 does not fire at decimation 10");
}

int main(void)
{
    test_decimation_of_zero_or_one_is_every_cycle();
    test_decimation_of_ten_fires_every_tenth_cycle();
    test_decimation_of_ten_skips_intermediate_cycles();

    printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
