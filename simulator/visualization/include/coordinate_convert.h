// NED (flight_core/simulator world frame) <-> VGL render-space coordinate conversion. VGL is a
// standard OpenGL right-handed, Y-up renderer (see ~/GitHub/VGL/README.md's example.cpp: grid
// lines and shapes are laid out in the X/Z plane with +Y as "up"), while this project's world
// frame is NED (README.md/AGENTS.md: X=forward/north, Y=right/east, Z=down) -- so every
// position/orientation drawn by scene_renderer.h/.cpp has to pass through here first.
//
// Deliberately glm/VGL-free: depends only on flight_core's Vec3/Quaternion, so this conversion is
// pure and host-testable the same way every other pure-logic module in this project is (AGENTS.md's
// driver-testing convention) -- see tests/coordinate_convert_test.cpp, which compiles
// coordinate_convert.cpp directly without ever touching VGL. scene_renderer.h/.cpp (which does
// depend on VGL/glm) converts RenderVec3/RenderQuat to glm::vec3/glm::quat at the call site.
//
// The conversion is a single fixed change of basis: (x, y, z) -> (x, -z, y), i.e. NED's forward
// axis (X) stays render space's X, and NED's -Z (up) becomes render space's +Y. This is the
// unique axis permutation that (a) preserves NED's forward axis as render space's horizontal
// "look" axis, (b) puts "up" on render space's +Y like every VGL example does, and (c) stays
// right-handed (X_new x Y_new = Z_new) rather than mirroring the scene -- see docs/visualization.md
// for the full derivation, including why it's the same fixed map as a +90 degree rotation about
// the shared X axis.
//
// Because that map is itself a proper rotation (right-handed, determinant +1), applying it to an
// orientation quaternion is just applying the same (x, y, z) -> (x, -z, y) permutation to the
// quaternion's vector part and leaving the scalar part (w) untouched -- conjugating a rotation
// quaternion by another rotation quaternion rotates only its axis, never its angle. See
// docs/visualization.md for the derivation of this too.
#pragma once

#include "quaternion.h"
#include "vec3.h"

namespace bicopter::visualization {

// A render-space position or direction. Plain (no glm dependency) so this header stays
// VGL-free -- see file header.
struct RenderVec3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// A render-space orientation, scalar-first [w, x, y, z] -- same storage order as
// flight_core's Quaternion.
struct RenderQuat {
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// Converts a NED position or direction vector to render space.
RenderVec3 nedToRender(const bicopter::Vec3& v);

// Converts a NED body-to-world orientation quaternion to the equivalent orientation in render
// space.
RenderQuat nedToRenderQuat(const bicopter::Quaternion& q);

} // namespace bicopter::visualization
