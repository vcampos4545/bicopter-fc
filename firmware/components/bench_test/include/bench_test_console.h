// UART I/O half of the bench-test console (Milestone 18 - see docs/bench_test.md). A minimal
// custom UART command parser was chosen over ESP-IDF's `esp_console` component specifically so
// the actual command grammar/validation (bench_test_command.h/.c) stays plain, ESP-IDF-free, and
// host-tested like every other parser in this project (radio_packet.c, crsf_frame.c), rather than
// depending on esp_console's own argtable3/linenoise machinery for logic this project wants to
// review and test itself - see docs/bench_test.md for the full tradeoff writeup.
//
// Uses ESP-IDF's UART driver in interrupt/ring-buffer mode (uart_driver_install()), the same
// pattern firmware/components/radio/src/crsf_radio.c already established for a UART-connected
// peripheral: process_pending() drains the driver's event queue non-blockingly once per caller
// cycle, so a task calling it never blocks waiting for console input and can keep feeding the task
// watchdog (or, for BenchTestTask specifically, simply isn't watchdog-registered at all - see
// bench_test_task.c). Defaults to a UART port distinct from both UART0 (logging/programming) and
// CRSF's own default (UART1) - see Kconfig.projbuild's "Bicopter bench-test tooling" menu.
#ifndef BICOPTER_BENCH_TEST_CONSOLE_H
#define BICOPTER_BENCH_TEST_CONSOLE_H

#include "driver/uart.h"
#include "esp_err.h"

#include "bench_test_command.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bench_test_console_dev *bench_test_console_handle_t;

typedef struct {
    uart_port_t uart_port;
    int rx_gpio;
    int tx_gpio;
    int baud_rate;
    int event_queue_length; // 0 defaults to 8, same convention as crsf_radio_config_t
} bench_test_console_config_t;

// Configures and starts the UART driver for the bench-test console. Never touches any actuator/
// sensor/radio state itself - this component only ever moves bytes and parsed commands, matching
// bench_test_command.h's own I/O-free scope one layer up.
esp_err_t bench_test_console_init(const bench_test_console_config_t *config,
                                   bench_test_console_handle_t *out_handle);

// Drains whatever is currently buffered from the UART, assembles it into lines, and for each
// complete line: echoes it back out the same UART (so an operator typing at a terminal sees what
// was received), calls bench_test_parse_line() on it, and invokes exactly one of `on_command`
// (parse succeeded with a non-NONE command) or `on_parse_error` (parse failed, or succeeded with
// BENCH_TEST_CMD_NONE - a blank line - which is silently ignored rather than calling either
// callback). Both callbacks may be NULL. Never blocks - mirrors
// crsf_radio_process_pending()/esp_now_radio_process_pending()'s non-blocking per-cycle drain
// pattern. `ctx` is passed through to whichever callback fires, unchanged.
esp_err_t bench_test_console_process_pending(
    bench_test_console_handle_t handle,
    void (*on_command)(const bench_test_command_t *cmd, void *ctx),
    void (*on_parse_error)(bench_test_parse_result_t result, const char *line, void *ctx),
    void *ctx);

// Writes `text` out the console UART followed by "\r\n". For streaming sensor/radio-print output
// and command-response messages. Not safe to call concurrently with itself or with
// process_pending() from a second task - this component assumes a single owning task, same as
// every other HAL-style handle in this project.
esp_err_t bench_test_console_println(bench_test_console_handle_t handle, const char *text);

esp_err_t bench_test_console_deinit(bench_test_console_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_BENCH_TEST_CONSOLE_H
