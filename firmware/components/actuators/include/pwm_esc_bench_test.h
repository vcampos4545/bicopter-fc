// BENCH_TEST compile-time motor-disable gate (Milestone 18 - see docs/bench_test.md).
// CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED only exists as a macro inside an ESP-IDF build (it
// comes from sdkconfig.h, generated from firmware/main/Kconfig.projbuild). This header gives it a
// plain, always-defined name - PWM_ESC_BENCH_TEST_MOTORS_DISABLED - so both pwm_esc_output.c (the
// ESP-IDF hardware-I/O half, which uses it to #if out the LEDC calls entirely) and a host test
// (tests/pwm_esc_bench_test_gate_test.c, which has no sdkconfig.h at all) can see the same
// definition without either depending on the other's build environment. When
// CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED is undefined (every non-ESP-IDF build, and every
// ESP-IDF build with the Kconfig option off - its default), this resolves to 0: "motors enabled,"
// the same default the Kconfig option itself has.
#ifndef BICOPTER_ACTUATORS_PWM_ESC_BENCH_TEST_H
#define BICOPTER_ACTUATORS_PWM_ESC_BENCH_TEST_H

#ifndef CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED
#define CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED 0
#endif

#define PWM_ESC_BENCH_TEST_MOTORS_DISABLED CONFIG_BICOPTER_BENCH_TEST_MOTORS_DISABLED

#endif // BICOPTER_ACTUATORS_PWM_ESC_BENCH_TEST_H
