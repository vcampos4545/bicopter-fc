# radio: ESP-NOW link (Milestone 14), the first concrete implementation of the protocol-
# independent Radio HAL interface (include/radio.h; a future RC-receiver implementation,
# Milestone 15, is a second backend behind the same interface). radio_packet.c/.h contains the
# pure wire-format/validation/sequence/loss-tracking logic and is also compiled standalone by
# ../../../tests/CMakeLists.txt for host-side unit testing, independent of ESP-IDF -- same split
# convention as firmware/components/sensors (see AGENTS.md's driver-testing convention).

idf_component_register(SRCS "src/radio_packet.c" "src/esp_now_radio.c"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_wifi esp_event esp_netif nvs_flash esp_timer)
