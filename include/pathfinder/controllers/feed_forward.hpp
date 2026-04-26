#pragma once

namespace pathfinder {

// kS is the static-friction breakaway voltage — small constant added in the
// direction of v_target to overcome static friction. Defaults to zero.
struct FeedForward {
    double kV = 0.0;
    double kA = 0.0;
    double kS = 0.0;

    double compute(double v_target, double a_target) const {
        const double s_term = (v_target == 0.0)
            ? 0.0
            : (v_target > 0.0 ? kS : -kS);
        return s_term + kV * v_target + kA * a_target;
    }
};

} // namespace pathfinder
