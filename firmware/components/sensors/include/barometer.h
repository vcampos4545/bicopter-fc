// Plain-C shape of the project's "Barometer" hardware-abstraction reading, per
// docs/architecture.md (poll-style `read() -> { pressure, temperature, timestamp }`), extended by
// this milestone to also carry a derived altitude and validity flag so a caller never has to
// re-derive altitude itself or guess whether a reading is usable (mirrors imu_reading_t's
// `valid` field). Deliberately free of ESP-IDF types so it compiles unmodified on the host (see
// tests/bmp581_convert_test.c) and stays part-agnostic: nothing here assumes BMP390 vs. BMP581 or
// any other specific chip (see docs/hardware.md). This header is the sensors component's local
// copy of that shape, not flight_core's eventual interface definition — flight_core is still an
// unimplemented stub as of this milestone.
#ifndef BICOPTER_SENSORS_BAROMETER_H
#define BICOPTER_SENSORS_BAROMETER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float pressure_pa;     // Pascals; smoothed per the driver's configured filter (see
                            // bmp581_convert.h) before altitude is derived from it
    float temperature_c;   // degrees Celsius, unfiltered
    float altitude_m;      // meters, standard barometric formula relative to the configured
                            // sea-level reference pressure (positive = above the reference)
    int64_t timestamp_us;  // monotonic microseconds, esp_timer_get_time()-compatible epoch
    bool valid;            // false if invalid: I2C failure. Callers must not treat pressure/
                            // temperature/altitude as real data when false.
} barometer_reading_t;

#ifdef __cplusplus
}
#endif

#endif // BICOPTER_SENSORS_BAROMETER_H
