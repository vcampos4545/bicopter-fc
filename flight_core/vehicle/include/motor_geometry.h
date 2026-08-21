// Shared motor-tilt geometry helper: the thrust-direction formula every layer that reasons about
// this bicopter's tilt-vectoring geometry needs. Milestone 9's forward dynamics
// (simulator/physics/bicopter_dynamics.cpp) and Milestone 12's control allocation
// (flight_core/control/control_allocator.cpp) both call this exact function rather than either
// re-deriving the formula or one target depending on the other's build output.
//
// This function originally lived in simulator/physics/ (where Milestone 9 first wrote it) and
// moved here at Milestone 12 specifically so flight_core/control/'s allocator could reuse the
// identical, already-tested implementation instead of duplicating it — flight_core/control/
// cannot depend on simulator/physics/ (simulator/physics/'s bicopter_physics target links
// flight_core, never the reverse), so shared code has to live on the flight_core side of that
// dependency edge. This is the same reasoning Milestone 9 already applied to VehicleParams (see
// vehicle_params.h's file header and docs/dynamics.md). tests/bicopter_dynamics_test.cpp's
// test_thrust_direction still exercises this exact function unmodified.
#pragma once

#include <cmath>

#include "vec3.h"

namespace bicopter {

// Direction (unit vector) of one motor's thrust vector in BODY frame at the given tilt angle: a
// rotation of tilt_rad about body +Y (right-hand rule) applied to the zero-tilt thrust axis
// (0, 0, -1) -- straight "up" in this project's Z-down body frame. theta = 0 gives straight up;
// positive theta tilts the thrust vector toward -X (aft). See docs/dynamics.md's "Tilt-vectoring
// geometry" section for the full derivation and sign convention, and docs/control_allocation.md
// for how the small-angle form of this exact formula is inverted for control allocation.
inline Vec3 motorThrustDirectionBody(float tilt_rad)
{
    return Vec3(-std::sin(tilt_rad), 0.0f, -std::cos(tilt_rad));
}

} // namespace bicopter
