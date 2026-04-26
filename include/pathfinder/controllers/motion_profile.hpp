#pragma once

#include <algorithm>
#include <cmath>

namespace pathfinder {
namespace motion_profile {

struct Sample {
    double position     = 0.0;
    double velocity     = 0.0;
    double acceleration = 0.0;
};

// Symmetric trapezoidal velocity profile (accel == decel == a_max).
// Handles both triangle (distance too short to reach v_max) and trapezoid cases.
// Reverse motion (end_pos < start_pos) handled by flipping sign at the end.
struct Trapezoidal {
    double v_max     = 0.0;
    double a_max     = 0.0;
    double start_pos = 0.0;
    double end_pos   = 0.0;

    double total_duration() const {
        const double D = std::abs(end_pos - start_pos);
        if (D <= 0.0 || v_max <= 0.0 || a_max <= 0.0) return 0.0;

        const double d_accel = (v_max * v_max) / (2.0 * a_max);
        if (2.0 * d_accel >= D) {
            const double v_peak = std::sqrt(D * a_max);
            return 2.0 * v_peak / a_max;
        }
        const double t_a = v_max / a_max;
        const double t_c = (D - 2.0 * d_accel) / v_max;
        return 2.0 * t_a + t_c;
    }

    Sample sample(double t) const {
        const double D = std::abs(end_pos - start_pos);
        const double sign = (end_pos >= start_pos) ? 1.0 : -1.0;

        if (D <= 0.0 || v_max <= 0.0 || a_max <= 0.0) {
            return {start_pos, 0.0, 0.0};
        }

        const double T = total_duration();
        if (t <= 0.0) return {start_pos, 0.0, 0.0};
        if (t >= T)   return {end_pos,   0.0, 0.0};

        const double d_accel = (v_max * v_max) / (2.0 * a_max);
        double pos_unsigned = 0.0;
        double vel_unsigned = 0.0;
        double acc_unsigned = 0.0;

        if (2.0 * d_accel >= D) {
            const double v_peak = std::sqrt(D * a_max);
            const double t_p    = v_peak / a_max;
            if (t < t_p) {
                acc_unsigned = a_max;
                vel_unsigned = a_max * t;
                pos_unsigned = 0.5 * a_max * t * t;
            } else {
                const double t2 = t - t_p;
                acc_unsigned = -a_max;
                vel_unsigned = v_peak - a_max * t2;
                pos_unsigned = 0.5 * v_peak * t_p
                             + v_peak * t2 - 0.5 * a_max * t2 * t2;
            }
        } else {
            const double t_a = v_max / a_max;
            const double t_c = (D - 2.0 * d_accel) / v_max;
            if (t < t_a) {
                acc_unsigned = a_max;
                vel_unsigned = a_max * t;
                pos_unsigned = 0.5 * a_max * t * t;
            } else if (t < t_a + t_c) {
                const double t2 = t - t_a;
                acc_unsigned = 0.0;
                vel_unsigned = v_max;
                pos_unsigned = d_accel + v_max * t2;
            } else {
                const double t2 = t - (t_a + t_c);
                acc_unsigned = -a_max;
                vel_unsigned = v_max - a_max * t2;
                pos_unsigned = d_accel + v_max * t_c
                             + v_max * t2 - 0.5 * a_max * t2 * t2;
            }
        }

        return {start_pos + sign * pos_unsigned,
                sign * vel_unsigned,
                sign * acc_unsigned};
    }
};

// 7-segment jerk-limited S-curve. Segments (constant jerk in each):
//   0: +j_max  (accel ramp up)
//   1:  0      (constant a_max)
//   2: -j_max  (accel ramp down)
//   3:  0      (cruise at v_max)
//   4: -j_max  (decel ramp up)
//   5:  0      (constant -a_max)
//   6: +j_max  (decel ramp down)
//
// If the distance is too short to fit all 7 segments at the configured limits,
// falls back to a trapezoidal profile with the same v_max/a_max.
struct SCurve {
    double v_max     = 0.0;
    double a_max     = 0.0;
    double j_max     = 0.0;
    double start_pos = 0.0;
    double end_pos   = 0.0;

private:
    struct Plan {
        bool   degenerate = false;
        double t_j        = 0.0;   // jerk-segment duration
        double t_a        = 0.0;   // constant-accel duration
        double t_v        = 0.0;   // cruise duration
        double T          = 0.0;
    };

    Plan plan() const {
        Plan p{};
        const double D = std::abs(end_pos - start_pos);
        if (D <= 0.0 || v_max <= 0.0 || a_max <= 0.0 || j_max <= 0.0) {
            p.degenerate = true;
            return p;
        }

        p.t_j = a_max / j_max;
        p.t_a = (v_max / a_max) - p.t_j;

        if (p.t_a < 0.0) {
            p.degenerate = true;
            return p;
        }

        const double d_accel_phase =
            v_max * (p.t_j + p.t_a);
        const double d_two_accels = 2.0 * d_accel_phase;

        if (d_two_accels > D) {
            p.degenerate = true;
            return p;
        }

        p.t_v = (D - d_two_accels) / v_max;
        p.T   = 4.0 * p.t_j + 2.0 * p.t_a + p.t_v;
        return p;
    }

    Trapezoidal fallback() const {
        return Trapezoidal{v_max, a_max, start_pos, end_pos};
    }

public:
    double total_duration() const {
        const Plan p = plan();
        if (p.degenerate) return fallback().total_duration();
        return p.T;
    }

    Sample sample(double t) const {
        const Plan p = plan();
        if (p.degenerate) return fallback().sample(t);

        const double sign = (end_pos >= start_pos) ? 1.0 : -1.0;
        if (t <= 0.0) return {start_pos, 0.0, 0.0};
        if (t >= p.T) return {end_pos,   0.0, 0.0};

        const double t_j = p.t_j;
        const double t_a = p.t_a;
        const double t_v = p.t_v;
        const double j   = j_max;
        const double a   = a_max;
        const double v   = v_max;

        const double t1 = t_j;
        const double t2 = t1 + t_a;
        const double t3 = t2 + t_j;
        const double t4 = t3 + t_v;
        const double t5 = t4 + t_j;
        const double t6 = t5 + t_a;

        // End-of-segment cached values.
        const double v_end_s0 = 0.5 * j * t_j * t_j;
        const double p_end_s0 = (1.0 / 6.0) * j * t_j * t_j * t_j;

        const double v_end_s1 = v_end_s0 + a * t_a;
        const double p_end_s1 = p_end_s0 + v_end_s0 * t_a + 0.5 * a * t_a * t_a;

        const double p_end_s2 = p_end_s1 + v_end_s1 * t_j
                              + 0.5 * a * t_j * t_j
                              - (1.0 / 6.0) * j * t_j * t_j * t_j;

        const double p_end_s3 = p_end_s2 + v * t_v;

        const double p_end_s4 = p_end_s3 + v * t_j
                              - (1.0 / 6.0) * j * t_j * t_j * t_j;

        const double p_end_s5 = p_end_s4 + v_end_s1 * t_a - 0.5 * a * t_a * t_a;

        double pos = 0.0, vel = 0.0, acc = 0.0;

        if (t < t1) {
            const double dt = t;
            acc = j * dt;
            vel = 0.5 * j * dt * dt;
            pos = (1.0 / 6.0) * j * dt * dt * dt;
        } else if (t < t2) {
            const double dt = t - t1;
            acc = a;
            vel = v_end_s0 + a * dt;
            pos = p_end_s0 + v_end_s0 * dt + 0.5 * a * dt * dt;
        } else if (t < t3) {
            const double dt = t - t2;
            acc = a - j * dt;
            vel = v_end_s1 + a * dt - 0.5 * j * dt * dt;
            pos = p_end_s1 + v_end_s1 * dt
                + 0.5 * a * dt * dt
                - (1.0 / 6.0) * j * dt * dt * dt;
        } else if (t < t4) {
            const double dt = t - t3;
            acc = 0.0;
            vel = v;
            pos = p_end_s2 + v * dt;
        } else if (t < t5) {
            const double dt = t - t4;
            acc = -j * dt;
            vel = v - 0.5 * j * dt * dt;
            pos = p_end_s3 + v * dt - (1.0 / 6.0) * j * dt * dt * dt;
        } else if (t < t6) {
            const double dt = t - t5;
            acc = -a;
            vel = v_end_s1 - a * dt;
            pos = p_end_s4 + v_end_s1 * dt - 0.5 * a * dt * dt;
        } else {
            const double dt = t - t6;
            acc = -a + j * dt;
            vel = v_end_s0 - a * dt + 0.5 * j * dt * dt;
            pos = p_end_s5 + v_end_s0 * dt
                - 0.5 * a * dt * dt
                + (1.0 / 6.0) * j * dt * dt * dt;
        }

        return {start_pos + sign * pos, sign * vel, sign * acc};
    }
};

} // namespace motion_profile
} // namespace pathfinder
