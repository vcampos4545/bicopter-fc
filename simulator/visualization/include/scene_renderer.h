// The VGL-dependent half of simulator/visualization/: draws one frame of a running SimLoop's
// state with VGL's GUI class -- vehicle body, both motor positions, live per-motor thrust
// vectors, and the desired/target attitude reference. See coordinate_convert.h for the NED
// <-> render-space conversion this relies on, and docs/visualization.md for what each primitive
// means and why.
//
// This header (and scene_renderer.cpp) is the only place in this project that includes
// <vgl/vgl.h> -- flight_core/, firmware/, and tests/ never do, and never link the `vgl` target
// (see ../CMakeLists.txt).
#pragma once

#include <vgl/vgl.h>

#include "sim_loop.h"
#include "vehicle_params.h"

namespace bicopter::visualization {

class SceneRenderer {
public:
    explicit SceneRenderer(const bicopter::VehicleParams& vehicle_params);

    // Draws one frame's worth of scene geometry into gui, reflecting loop's current true state,
    // last-allocated actuator command, and desired attitude. Must be called between
    // gui.beginFrame()/gui.endFrame() -- SceneRenderer owns no window/frame state of its own,
    // same "caller owns the loop" division VGL's own examples use.
    void render(GUI& gui, const bicopter::SimLoop& loop) const;

private:
    bicopter::VehicleParams vehicle_params_;

    void drawGroundGrid(GUI& gui) const;
    void drawMotor(GUI& gui, const bicopter::MotorParams& motor, float throttle, float tilt_rad,
                    const bicopter::Vec3& body_pos_ned, const bicopter::Quaternion& body_orientation) const;
};

} // namespace bicopter::visualization
