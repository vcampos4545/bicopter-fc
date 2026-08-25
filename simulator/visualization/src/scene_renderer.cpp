#include "scene_renderer.h"

#include "coordinate_convert.h"
#include "control_allocator.h"
#include "motor_geometry.h"

namespace bicopter::visualization {

namespace {

glm::vec3 toGlm(const RenderVec3& v)
{
    return glm::vec3(v.x, v.y, v.z);
}

glm::quat toGlm(const RenderQuat& q)
{
    return glm::quat(q.w, q.x, q.y, q.z);
}

// Same fixed NED->render axis permutation coordinate_convert.h uses for positions/directions,
// applied to a box's (always-positive) half-of-nothing-in-particular extents -- a size has no
// direction to flip sign on, just axes to relabel, so this drops nedToRender()'s sign flip and
// keeps only the permutation.
glm::vec3 nedSizeToRenderSize(const bicopter::Vec3& size_ned)
{
    return glm::vec3(size_ned.x, size_ned.z, size_ned.y);
}

// Vehicle body box extents (render-space X=forward, Y=up, Z=right), a fixed visual size
// independent of VehicleParams -- purely cosmetic, not a real airframe dimension.
const bicopter::Vec3 kBodyBoxSizeNed{0.30f, 0.10f, 0.08f}; // (forward, right, down) in NED

constexpr float kMotorSphereRadiusM = 0.035f;

// Thrust arrow length per Newton of motor thrust -- chosen so a roughly-hover-trim thrust
// (motor.thrust_coefficient_n * 0.5^2, a few Newtons for this project's demo vehicle fixture,
// see tests/sim_loop_test.cpp's baseParams()) draws an arrow comparable in length to the body
// box above, not so long it dwarfs the scene.
constexpr float kThrustLineScaleMPerN = 0.03f;

constexpr float kTargetTriadLengthM = 0.35f;

} // namespace

SceneRenderer::SceneRenderer(const bicopter::VehicleParams& vehicle_params)
    : vehicle_params_(vehicle_params)
{
}

void SceneRenderer::drawGroundGrid(GUI& gui) const
{
    constexpr int kNumLines = 20;
    constexpr float kSpacingM = 0.5f;
    constexpr float kHalfExtent = kNumLines * kSpacingM / 2.0f;
    const glm::vec3 grid_color(0.35f, 0.35f, 0.35f);

    for (int i = 0; i <= kNumLines; ++i) {
        const float offset = i * kSpacingM - kHalfExtent;
        gui.drawLine({-kHalfExtent, 0.0f, offset}, {kHalfExtent, 0.0f, offset}, grid_color, 1.0f);
        gui.drawLine({offset, 0.0f, -kHalfExtent}, {offset, 0.0f, kHalfExtent}, grid_color, 1.0f);
    }
}

void SceneRenderer::drawMotor(GUI& gui, const bicopter::MotorParams& motor, float throttle,
                                float tilt_rad, const bicopter::Vec3& body_pos_ned,
                                const bicopter::Quaternion& body_orientation) const
{
    // Motor position relative to the body's geometric reference origin (body frame) -- same
    // (0, arm_offset_y_m, 0) convention simulator/physics/src/bicopter_dynamics.cpp's
    // computeMotorEffect() uses for its torque-arm calculation.
    const bicopter::Vec3 motor_offset_body(0.0f, motor.arm_offset_y_m, 0.0f);
    const bicopter::Vec3 motor_pos_ned = body_pos_ned + body_orientation.rotate(motor_offset_body);
    const glm::vec3 motor_pos_render = toGlm(nedToRender(motor_pos_ned));

    const float throttle_clamped = throttle < 0.0f ? 0.0f : (throttle > 1.0f ? 1.0f : throttle);
    const glm::vec3 motor_color =
        glm::mix(glm::vec3(0.7f, 0.7f, 0.7f), glm::vec3(1.0f, 0.15f, 0.05f), throttle_clamped);
    gui.drawSphere(motor_pos_render, kMotorSphereRadiusM, motor_color);

    const bicopter::Vec3 thrust_dir_body = bicopter::motorThrustDirectionBody(tilt_rad);
    const bicopter::Vec3 thrust_dir_ned = body_orientation.rotate(thrust_dir_body);
    const glm::vec3 thrust_dir_render = toGlm(nedToRender(thrust_dir_ned));

    const float thrust_n = motor.thrust_coefficient_n * throttle_clamped * throttle_clamped;
    const glm::vec3 thrust_end = motor_pos_render + thrust_dir_render * (thrust_n * kThrustLineScaleMPerN);
    gui.drawLine(motor_pos_render, thrust_end, motor_color, 2.5f);
}

void SceneRenderer::render(GUI& gui, const bicopter::SimLoop& loop) const
{
    drawGroundGrid(gui);

    const bicopter::RigidBodyState& state = loop.trueState();
    const bicopter::AllocatedCommand& command = loop.lastLog().command;

    const glm::vec3 body_pos_render = toGlm(nedToRender(state.position_m));
    const glm::quat body_rot_render = toGlm(nedToRenderQuat(state.orientation));

    gui.drawBox(body_pos_render, nedSizeToRenderSize(kBodyBoxSizeNed), body_rot_render,
                glm::vec3(0.2f, 0.55f, 1.0f));

    drawMotor(gui, vehicle_params_.motor1, command.motor1_throttle, command.motor1_tilt_rad,
              state.position_m, state.orientation);
    drawMotor(gui, vehicle_params_.motor2, command.motor2_throttle, command.motor2_tilt_rad,
              state.position_m, state.orientation);

    // Target/desired attitude, rendered as a distinctly colored axis triad at the vehicle's
    // current position: when the vehicle's true attitude converges to the desired one, the body
    // box's edges line up with this triad -- that visible alignment IS the convergence
    // demonstration this task asked for, not a separate metric.
    const glm::quat target_rot_render = toGlm(nedToRenderQuat(loop.desiredAttitude()));
    const glm::vec3 target_x = target_rot_render * glm::vec3(kTargetTriadLengthM, 0.0f, 0.0f);
    const glm::vec3 target_y = target_rot_render * glm::vec3(0.0f, kTargetTriadLengthM, 0.0f);
    const glm::vec3 target_z = target_rot_render * glm::vec3(0.0f, 0.0f, kTargetTriadLengthM);
    gui.drawLine(body_pos_render, body_pos_render + target_x, glm::vec3(1.0f, 0.9f, 0.1f), 3.0f);
    gui.drawLine(body_pos_render, body_pos_render + target_y, glm::vec3(1.0f, 0.9f, 0.1f), 3.0f);
    gui.drawLine(body_pos_render, body_pos_render + target_z, glm::vec3(1.0f, 0.9f, 0.1f), 3.0f);
}

} // namespace bicopter::visualization
