// Real automated tests for simulator/visualization/coordinate_convert.{h,cpp}: the NED <-> VGL
// render-space conversion. Compiles coordinate_convert.cpp directly (no VGL/glm dependency, see
// its file header) the same way vec3_test.cpp links flight_core directly -- see
// simulator/visualization/CMakeLists.txt's comment for why this test never links the `vgl`
// target. Same hand-rolled assert-and-report harness as tests/vec3_test.cpp.

#include <cmath>
#include <cstdio>

#include "coordinate_convert.h"
#include "quaternion.h"
#include "vec3.h"

using bicopter::Quaternion;
using bicopter::Vec3;
using bicopter::visualization::nedToRender;
using bicopter::visualization::nedToRenderQuat;
using bicopter::visualization::RenderVec3;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, msg)                                                                         \
    do {                                                                                          \
        g_checks++;                                                                               \
        if (!(cond)) {                                                                            \
            g_failures++;                                                                         \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, msg);                             \
        }                                                                                          \
    } while (0)

#define CHECK_NEAR(a, b, tol, msg) CHECK(std::fabs((a) - (b)) <= (tol), msg)

static void test_position_conversion()
{
    // NED forward (X) stays render-space X.
    const RenderVec3 forward = nedToRender(Vec3(1.0f, 0.0f, 0.0f));
    CHECK_NEAR(forward.x, 1.0f, 1e-6f, "NED +X (forward) -> render +X");
    CHECK_NEAR(forward.y, 0.0f, 1e-6f, "NED +X (forward) -> render y=0");
    CHECK_NEAR(forward.z, 0.0f, 1e-6f, "NED +X (forward) -> render z=0");

    // NED up (-Z, since NED Z is down) becomes render-space +Y (VGL's "up").
    const RenderVec3 up = nedToRender(Vec3(0.0f, 0.0f, -1.0f));
    CHECK_NEAR(up.x, 0.0f, 1e-6f, "NED -Z (up) -> render x=0");
    CHECK_NEAR(up.y, 1.0f, 1e-6f, "NED -Z (up) -> render +Y");
    CHECK_NEAR(up.z, 0.0f, 1e-6f, "NED -Z (up) -> render z=0");

    // NED right (+Y) becomes render-space +Z.
    const RenderVec3 right = nedToRender(Vec3(0.0f, 1.0f, 0.0f));
    CHECK_NEAR(right.x, 0.0f, 1e-6f, "NED +Y (right) -> render x=0");
    CHECK_NEAR(right.y, 0.0f, 1e-6f, "NED +Y (right) -> render y=0");
    CHECK_NEAR(right.z, 1.0f, 1e-6f, "NED +Y (right) -> render +Z");

    // Linearity: an arbitrary vector converts component-wise via the same fixed map.
    const RenderVec3 v = nedToRender(Vec3(2.0f, -3.0f, 4.0f));
    CHECK_NEAR(v.x, 2.0f, 1e-6f, "arbitrary vector x");
    CHECK_NEAR(v.y, -4.0f, 1e-6f, "arbitrary vector y = -ned.z");
    CHECK_NEAR(v.z, -3.0f, 1e-6f, "arbitrary vector z = ned.y");
}

static void test_conversion_is_right_handed()
{
    // X_render cross Y_render must equal Z_render for the converted basis to stay right-handed
    // (no mirroring) -- verified on the three converted NED basis vectors themselves.
    const RenderVec3 rx = nedToRender(Vec3(1, 0, 0));
    const RenderVec3 ry = nedToRender(Vec3(0, 0, -1)); // NED up
    const RenderVec3 rz = nedToRender(Vec3(0, 1, 0));  // NED right

    // Cross product rx x ry, computed by hand since RenderVec3 has no operator support.
    const float cx = rx.y * ry.z - rx.z * ry.y;
    const float cy = rx.z * ry.x - rx.x * ry.z;
    const float cz = rx.x * ry.y - rx.y * ry.x;

    CHECK_NEAR(cx, rz.x, 1e-6f, "converted basis stays right-handed (x)");
    CHECK_NEAR(cy, rz.y, 1e-6f, "converted basis stays right-handed (y)");
    CHECK_NEAR(cz, rz.z, 1e-6f, "converted basis stays right-handed (z)");
}

static void test_identity_quaternion_converts_to_identity()
{
    const auto q = nedToRenderQuat(Quaternion::Identity());
    CHECK_NEAR(q.w, 1.0f, 1e-6f, "identity quaternion w");
    CHECK_NEAR(q.x, 0.0f, 1e-6f, "identity quaternion x");
    CHECK_NEAR(q.y, 0.0f, 1e-6f, "identity quaternion y");
    CHECK_NEAR(q.z, 0.0f, 1e-6f, "identity quaternion z");
}

static void test_quaternion_rotation_angle_preserved()
{
    // Converting a rotation must not change its angle (only its axis, per this project's
    // right-handed change-of-basis) -- a 40 degree rotation about an arbitrary NED axis should
    // still be a 40 degree rotation in render space, i.e. |w| unchanged.
    const Vec3 axis = Vec3(1.0f, 2.0f, -3.0f).normalized();
    const float half_angle = (40.0f * 3.14159265f / 180.0f) / 2.0f;
    const Quaternion q_ned(std::cos(half_angle), axis.x * std::sin(half_angle),
                            axis.y * std::sin(half_angle), axis.z * std::sin(half_angle));

    const auto q_render = nedToRenderQuat(q_ned);
    CHECK_NEAR(q_render.w, q_ned.w, 1e-6f, "rotation angle (scalar part) unchanged by conversion");

    // The converted axis part must still be unit-length (scaled by sin(half_angle)), i.e. the
    // permutation didn't introduce any scaling.
    const float converted_axis_len =
        std::sqrt(q_render.x * q_render.x + q_render.y * q_render.y + q_render.z * q_render.z);
    const float original_axis_len = std::sqrt(q_ned.x * q_ned.x + q_ned.y * q_ned.y + q_ned.z * q_ned.z);
    CHECK_NEAR(converted_axis_len, original_axis_len, 1e-6f,
               "converted quaternion's axis part preserves magnitude");
}

int main()
{
    test_position_conversion();
    test_conversion_is_right_handed();
    test_identity_quaternion_converts_to_identity();
    test_quaternion_rotation_angle_preserved();

    std::printf("%d/%d checks passed\n", g_checks - g_failures, g_checks);
    return g_failures == 0 ? 0 : 1;
}
