# Milestone 18: bench-test tooling (see ../../../docs/bench_test.md). bench_test_command.c is
# pure, ESP-IDF-free line-parsing logic, host-tested directly (../../../tests/CMakeLists.txt),
# same driver-testing-convention split as every other component here (see AGENTS.md).
# bench_test_console.c is the ESP-IDF UART-driver half.
idf_component_register(SRCS "src/bench_test_command.c"
                             "src/bench_test_console.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver)
