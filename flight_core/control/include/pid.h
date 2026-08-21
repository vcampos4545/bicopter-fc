// Generic PID controller building block for flight_core/control/. Platform-independent, no
// ESP-IDF dependency (per AGENTS.md's flight_core rule) — this type is reused at different rates
// by both the rate controller (Milestone 10, rate_controller.h) and the future attitude
// controller (Milestone 11), which is why dt is passed explicitly per update() call rather than
// baked in as a fixed period internal to this type.
//
// Anti-windup: clamping (conditional integration), not back-calculation. update() always computes
// a tentative integrated error; if the resulting *unsaturated* output would exceed
// [output_min, output_max] and the current error would push it further into that saturation, the
// integral term is held at its previous value instead of accumulating further (it's still free to
// shrink back once the error reverses sign or the output un-saturates). Clamping was chosen over
// back-calculation because it needs no extra tracking-time-constant tuning parameter — it
// deterministically stops runaway integral growth using only the output_min/output_max limits
// this type already requires for saturation, keeping this generic building block's config surface
// minimal. See docs/control.md.
//
// Derivative convention: derivative-on-measurement (default, PidConfig::derivative_on_measurement
// = true), not derivative-on-error. Differentiating the measurement instead of
// (setpoint - measurement) avoids "derivative kick": a step change in setpoint makes the error
// jump discontinuously, and differentiating that jump produces a large, physically meaningless
// output spike. The measurement itself never jumps discontinuously, so differentiating it stays
// smooth across setpoint changes. See docs/control.md.
#pragma once

namespace bicopter {

struct PidConfig {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;

    // Output saturation limits. Callers must ensure output_min <= output_max.
    float output_min = -1.0f;
    float output_max = 1.0f;

    // true (default): differentiate the measurement. false: differentiate the error
    // (setpoint - measurement) instead. See the file header for why derivative-on-measurement is
    // the default.
    bool derivative_on_measurement = true;
};

class Pid {
public:
    explicit Pid(const PidConfig& config = {});

    // Clears integral accumulation and derivative history. Callers should call this whenever the
    // control loop is (re)armed, or after a large gap in update() calls where stale
    // integral/derivative state would otherwise produce a bogus transient.
    void reset();

    // Runs one control step and returns the saturated output. dt is the elapsed time in seconds
    // since the previous update() call (or since construction/reset(), for the first call) —
    // never assumed fixed, so one Pid instance can be reused at different call rates. dt <= 0
    // skips integration and differentiation for this call (returns the clamped proportional-only
    // response) rather than dividing by zero or a negative dt.
    float update(float setpoint, float measurement, float dt);

    const PidConfig& config() const { return config_; }
    void setConfig(const PidConfig& config) { config_ = config; }

    // Exposed for tests/telemetry: the most recent call's pre-saturation output and the current
    // integral accumulator, so anti-windup behavior can be verified directly rather than only
    // inferred from the clamped output.
    float lastUnsaturatedOutput() const { return last_unsaturated_output_; }
    float integralTerm() const { return integral_; }

private:
    PidConfig config_;

    float integral_ = 0.0f;
    float previous_measurement_ = 0.0f;
    float previous_error_ = 0.0f;
    bool have_previous_ = false;

    float last_unsaturated_output_ = 0.0f;
};

} // namespace bicopter
