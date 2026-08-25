# Simulator graphical visualization

A follow-up task, not one of the 18 numbered milestones (see AGENTS.md/TODO.md): a real graphical
renderer of Milestone 13's closed loop (`simulator/sim_loop/`'s `SimLoop`), built on the captain's
[VGL](https://github.com/vcampos4545/VGL) — a minimal OpenGL rendering layer (`GUI` class:
`drawSphere`/`drawCube`/`drawBox`/`drawLine`, quaternion-rotated overloads, an orbit camera,
keyboard/mouse input). `simulator/CMakeLists.txt`'s original Milestone 13 comment reserved
`simulator/visualization/` for exactly this; this task fills it in for real, additively —
`simulator/main.cpp`'s existing text-trace `bicopter_sim` demo is unchanged and still builds.

## What it renders

`bicopter_sim_viz` opens a window and, every frame, draws `SimLoop`'s current state:

- **Vehicle body** — a box at `RigidBodyState::position_m`, rotated by `RigidBodyState::orientation`
  (VGL's quaternion-rotated `drawBox` overload).
- **Both motor positions** — small spheres at each motor's body-frame offset
  (`(0, MotorParams::arm_offset_y_m, 0)`, the same reference-origin convention
  `simulator/physics/src/bicopter_dynamics.cpp`'s `computeMotorEffect()` uses), rotated and
  translated with the body. Sphere color interpolates from grey (zero throttle) to red (full
  throttle).
- **Live per-motor thrust vectors** — a line from each motor position along
  `motorThrustDirectionBody()`'s direction for that motor's current commanded tilt
  (`flight_core/vehicle/include/motor_geometry.h`), rotated into world frame by the body
  orientation. Length scales with `thrust_coefficient_n * throttle^2` (the same thrust model
  `VehicleParams`/`ControlAllocator` use) at a fixed, purely cosmetic
  `kThrustLineScaleMPerN` (`scene_renderer.cpp`).
- **Target/desired attitude** — a distinctly colored (yellow) axis triad at the vehicle's current
  position, oriented by `SimLoop::desiredAttitude()`. As the vehicle's true attitude converges,
  the body box's edges visibly align with this triad — that alignment *is* the convergence
  demonstration, not a separate metric.
- **A ground grid** (`drawLine`, not VGL's `drawInfiniteGroundPlane` — see "VGL coordinate
  convention" below for why) and a console readout of `true_attitude_error_rad`/
  `estimated_attitude_error_rad` printed periodically, the same metric
  `tests/sim_loop_test.cpp`/`simulator/main.cpp` already use.

Camera: VGL's `OrbitalCamera` — left-drag to orbit, right-drag to pan, scroll to zoom.

## Coordinate convention: NED vs. VGL render space

flight_core/this project's world frame is **NED** (README.md/AGENTS.md: X=forward, Y=right,
Z=down). VGL is a standard OpenGL **right-handed, Y-up** renderer — its own examples
(`~/GitHub/VGL/examples/example.cpp`) lay shapes out in the X/Z plane with +Y as "up". Every
position/orientation this renderer draws has to cross that boundary, so
`simulator/visualization/include/coordinate_convert.h` defines the one fixed conversion used
everywhere:

```
(x, y, z)_NED -> (x, -z, y)_render
```

i.e. NED's forward axis (X) stays render space's X, and NED's -Z (straight up) becomes render
space's +Y. This is the unique axis permutation that (a) keeps NED's forward axis as render
space's horizontal "look" axis, (b) puts "up" on render space's +Y like every VGL example does,
and (c) stays right-handed — X_render × Y_render = Z_render — rather than mirroring the scene.
Right-handedness check, using the three converted NED basis vectors: converting
`X=(1,0,0)`, `up=(0,0,-1)`, and `right=(0,1,0)` gives `(1,0,0)`, `(0,1,0)`, `(0,0,1)` respectively,
and `(1,0,0) × (0,1,0) = (0,0,1)` — confirmed. `tests/coordinate_convert_test.cpp` asserts this
directly.

Because that map is itself a proper rotation (right-handed, determinant +1 — concretely, a fixed
+90 degree rotation about the shared X axis), converting an orientation quaternion is *not* a
separate derivation: conjugating a rotation quaternion by another rotation quaternion rotates only
its axis, never its angle, so `nedToRenderQuat()` just applies the same `(x, y, z) -> (x, -z, y)`
permutation to the quaternion's vector part and leaves the scalar part (`w`) untouched.
`tests/coordinate_convert_test.cpp`'s `test_quaternion_rotation_angle_preserved` checks this
(rotation angle/`w` unchanged, axis-part magnitude unchanged) for an arbitrary rotation, not just
the identity case.

This is also why the renderer doesn't use VGL's `drawInfiniteGroundPlane()`: reading its shader
(`~/GitHub/VGL/include/vgl/EmbeddedShaders.h`'s `groundPlaneFrag`) shows it ray-casts against a
plane at world **Z** = `planeZ`, i.e. that one function treats Z as "up" — inconsistent with the
Y-up convention every other VGL draw call and example uses. Rather than mixing two different "up"
axes in one scene, `SceneRenderer::drawGroundGrid()` draws a plain Y=0 grid of lines instead
(the same approach `~/GitHub/VGL/examples/example.cpp` uses).

### Why this conversion is pure and host-tested

`coordinate_convert.h`/`.cpp` depends only on `flight_core`'s `Vec3`/`Quaternion` — no glm, no VGL
— by design, so it follows AGENTS.md's driver-testing convention the same way every hardware
driver's pure conversion logic does: it's compiled directly into
`tests/coordinate_convert_test.cpp` (see `tests/CMakeLists.txt`), which never links the `vgl`
target. `simulator/visualization/include/scene_renderer.h`/`.cpp` is the VGL-dependent half that
actually issues draw calls, converting `RenderVec3`/`RenderQuat` to `glm::vec3`/`glm::quat` at the
call site — it's the only place in this project that includes `<vgl/vgl.h>`.

## Real closed loop, real-time pacing

`bicopter_sim_viz` runs the identical scenario `simulator/main.cpp`'s text-trace `bicopter_sim`
demonstrates: the same vehicle fixture, same 15° roll + 20° yaw disturbance, same estimator/noise
configuration (`demoVehicleParams()`, duplicated in `main_viz.cpp` the same way `main.cpp` already
duplicates `tests/sim_loop_test.cpp`'s `baseParams()` — see that file's own comment for why this
project accepts that small duplication rather than sharing a demo-only helper across a test/binary
boundary). `SimLoop::step()` runs at its own fixed `physics_dt_s` rate (default 2 ms / 500 Hz),
independent of render frame rate: each rendered frame accumulates real elapsed wall-clock time
(`std::chrono::steady_clock`) and drains it in fixed `physics_dt_s` steps — a standard
fixed-timestep accumulator (frame time is clamped to 0.1s so a paused/breakpointed frame doesn't
dump a huge non-realtime burst of steps). This means simulated time tracks wall-clock time, and
the window's visible behavior (and convergence time) doesn't depend on the display's refresh rate.

## Build

VGL is pulled in via CMake `FetchContent` (its README's documented recommended approach),
`GIT_REPOSITORY https://github.com/vcampos4545/VGL.git`, scoped *entirely* to
`simulator/visualization/CMakeLists.txt` — `flight_core/`, `firmware/`, and `tests/` never
`add_subdirectory` that directory and never link the `vgl` target, so none of them gain a VGL
dependency. `simulator/CMakeLists.txt` gates `add_subdirectory(visualization)` behind a
`BICOPTER_SIM_BUILD_VISUALIZATION` CMake option (default `ON`) so a machine without VGL's
dependencies, or without network access for the `FetchContent` clone, can still configure/build
the rest of `simulator/` (including the existing `bicopter_sim`) with that option turned off.

Requires VGL's own stated dependencies:

```sh
# macOS
brew install glfw glew glm

# Ubuntu
sudo apt install libglfw3-dev libglew-dev libglm-dev
```

Confirmed present in this project's development environment as `glfw 3.4`, `glew 2.2.0_1`,
`glm 1.0.2` (Homebrew).

```sh
cmake -S simulator -B simulator/build
cmake --build simulator/build
./simulator/build/bicopter_sim                        # existing text-trace demo, unaffected
./simulator/build/visualization/bicopter_sim_viz       # new graphical visualizer

# Without VGL/its dependencies available:
cmake -S simulator -B simulator/build -DBICOPTER_SIM_BUILD_VISUALIZATION=OFF
cmake --build simulator/build
./simulator/build/bicopter_sim                         # still builds and runs
```

`tests/CMakeLists.txt` adds `coordinate_convert_test` (compiling
`simulator/visualization/src/coordinate_convert.cpp` directly, linking `flight_core` — never
`vgl`), alongside the existing suite:

```sh
cmake -S tests -B tests/build
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
```

`firmware/` is completely untouched by this task — no file under `firmware/` or `flight_core/`
changed, and `idf.py build` was confirmed to still succeed.

## Verification: what's actually confirmed vs. what's honest-eyeball-only

Per this project's established convention (every hardware-adjacent milestone is explicit about
verified-vs-deferred, see e.g. docs/bench_test.md, docs/radio.md), this task is inherently not
host-testable in the usual automated sense — it opens a real window. What's actually verified:

- **Automated / host-tested**: `coordinate_convert_test` (13 checks) — the NED<->render-space
  position and quaternion conversion, including the right-handedness check and the
  rotation-angle-preservation check described above. All pre-existing `tests/` targets (27 total)
  and `simulator/`'s existing `bicopter_sim` still build and pass unmodified.
- **Confirmed by build, not by eye**: `cmake -S simulator -B simulator/build && cmake --build
  simulator/build` succeeds and produces both `bicopter_sim` and `bicopter_sim_viz` (VGL fetched
  and linked correctly); `idf.py build` succeeds against unmodified `firmware/`.
  `-DBICOPTER_SIM_BUILD_VISUALIZATION=OFF` was also confirmed to configure/build `bicopter_sim`
  successfully without ever fetching VGL.
- **Confirmed by running it (manual, this environment)**: `bicopter_sim_viz` was launched and ran
  for 30+ seconds without crashing, opening a real GLFW/OpenGL window. Its console readout (the
  same `true_attitude_error_rad`/`estimated_attitude_error_rad` trace `bicopter_sim` prints)
  showed the closed loop converging from a 24.95° initial disturbance to a settled ~4.64° residual
  error within about 1.4 simulated seconds and holding stably there for the rest of the run —
  matching `tests/sim_loop_test.cpp`'s `test_roll_yaw_disturbance_converges` case (same scenario,
  6° tolerance) almost exactly. Simulated time (`t=`) tracked wall-clock elapsed time throughout
  the run, confirming the fixed-timestep accumulator paces the simulation in real time rather than
  fast-forwarding or stalling. This is strong evidence the real `SimLoop`/estimator/controller/
  allocator stack is genuinely running (not a scripted animation) and that the values being fed to
  every draw call (`SceneRenderer::render()`) are the real, converging closed-loop state.
- **Not confirmed by eye in this environment**: a screenshot of the actual rendered primitives
  (the body box, motor spheres, thrust-vector lines, target triad, and grid visibly moving and
  converging) could not be captured here — this task ran in an automated CLI environment without
  an interactive WindowServer-visible desktop session, so `screencapture` returned the desktop
  background rather than the GLFW window's contents, even while the process was confirmed running.
  The window creation, draw calls, and shape math are all exercised by the same running process
  that produced the convergence trace above (a crash or an OpenGL context/shader failure would
  have aborted that process, not silently kept printing a converging trace), and every geometric
  transform feeding those draw calls is unit-tested (`coordinate_convert_test`), but the final
  "does it look right on screen" check is left to whoever runs `bicopter_sim_viz` interactively.
