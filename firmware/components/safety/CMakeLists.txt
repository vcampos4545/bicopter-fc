# safety: flight-mode state machine, arming preconditions, and failsafe detection/response
# (Milestone 16). Every file here is ESP-IDF-free pure logic by design (see flight_mode.h's file
# header for why this whole component - not just a *_convert.c split within it - is pure, unlike
# sensors/actuators/radio's driver+pure-logic pairing) and is also compiled standalone by
# ../../../tests/CMakeLists.txt for host-side unit testing, independent of ESP-IDF.

idf_component_register(SRCS "src/flight_mode.c" "src/arming.c" "src/failsafe.c"
                             "src/actuator_command_check.c"
                        INCLUDE_DIRS "include")
