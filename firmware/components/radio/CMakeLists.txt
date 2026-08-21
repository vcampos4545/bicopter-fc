# radio: ESP-NOW link (Milestone 14) plus CRSF RC-receiver support (Milestone 15), two concrete
# implementations of the protocol-independent Radio HAL interface (include/radio.h). radio_packet.c
# and crsf_frame.c contain the pure wire-format/validation/sequence-or-frame-sync/loss-tracking
# logic and are also compiled standalone by ../../../tests/CMakeLists.txt for host-side unit
# testing, independent of ESP-IDF -- same split convention as firmware/components/sensors (see
# AGENTS.md's driver-testing convention). crsf_radio.c additionally needs the `driver` component
# for ESP-IDF's UART driver (uart_driver_install() et al.) -- see docs/radio.md.

idf_component_register(SRCS "src/radio_packet.c" "src/esp_now_radio.c" "src/crsf_frame.c"
                             "src/crsf_radio.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_timer driver)
