# power: ADC-based battery voltage (and optional current) monitoring (Milestone 16).
# battery_convert.c contains the pure raw-ADC-millivolts-to-voltage/percentage/threshold logic and
# is also compiled standalone by ../../../tests/CMakeLists.txt for host-side unit testing,
# independent of ESP-IDF - same split convention as firmware/components/sensors (see AGENTS.md's
# driver-testing convention). battery.c is the ESP-IDF `esp_adc` oneshot-driver I/O, not
# host-testable.

idf_component_register(SRCS "src/battery.c" "src/battery_convert.c"
                        INCLUDE_DIRS "include"
                        REQUIRES driver esp_adc)
