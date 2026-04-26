#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cassert>
#include <cmath>

namespace pathfinder {

// 6-DOF body-frame velocity: companion to the integrated `Pose2` state.
// Sign convention matches the global frame: +X forward, +Y right (per
// CLAUDE.md and SPEC.md §3). `omega_dps` is positive in the CW-from-above
// (compass-bearing) convention, same as `Pose2::heading`.
struct BodyVelocity {
    double v_x_ips   = 0.0;   // forward velocity in body frame
    double v_y_ips   = 0.0;   // lateral velocity in body frame (positive = +Y = right)
    double omega_dps = 0.0;   // heading rate (positive in our CW-from-above convention)
};

// Tier-1 localization: integrates wheel-encoder displacements + an absolute
// IMU heading into a Pose2 (Wave A) AND tracks a body-frame velocity triple
// `(v_x, v_y, ω)` (Wave D — spec step 12).
//
// Pose math (per spec App. A) — unchanged from Wave A:
//   For each step, the wheel center at body-frame (x_p, y_p) on a body whose
//   center translates by (d_x, d_y) and rotates by Δθ moves by
//     (d_x − Δθ · y_p,  d_y + Δθ · x_q)
//   in body frame. The parallel wheel reads body-X, the perp wheel reads
//   body-Y. Solving:
//     d_x = Δs_par  + Δθ · y_p
//     d_y = Δs_perp − Δθ · x_q       (or model-driven if no perp wheel)
//   The body-frame displacement is rotated into world frame using the heading
//   at the midpoint of the step (better than start- or end-heading for
//   integrating curved motion — second-order accurate Heun-like step).
//
// Velocity math (Wave D / spec §8 process model):
//   v_x   = d_x / dt   (always direct from the parallel-wheel delta)
//   ω_dps = (Δθ / dt) · 180/π
//   v_y:
//     • If has_perp_wheel: v_y = d_y / dt (direct measurement, low noise).
//     • Else (no perp wheel): forward-Euler the friction-decay + cross-coupling
//       process model from spec §8:
//           v_y_dot = −μ · v_y_prev − ω · v_x
//           v_y     = v_y_prev + v_y_dot · dt
//       where μ = lateral_friction_coefficient (units: 1/s). For all-omni
//       drivetrains μ ∈ [0.5, 1.5]; for traction wheels μ ∈ [5, 20] — drift
//       decays in tens of milliseconds and the body-frame v_y is effectively
//       always 0. Defaults conservative (5.0).
//
// Note: body-frame d_y for pose integration uses the wheel-derived value
// (perp encoder reading minus rotation contribution, or 0 if no perp wheel).
// The model-driven v_y is a *separate* estimate exposed to drift-aware
// controllers; it does NOT feed back into pose integration. DR alone has no
// way to distinguish "wheels measured zero lateral motion" from "lateral
// drift was real but unobserved" — the EKF (Wave F) is where the model and
// the measurement are fused into a single posterior. For now, pose tracks
// the wheels and the controllers compensate via the model-driven v_y.
class DeadReckoning {
public:
    struct Config {
        // Body-frame Y coord of the parallel tracking wheel's contact point.
        // 0 if the wheel sits on the bot's centerline (or if there's no
        // dedicated parallel wheel and the caller passes 0 deltas).
        double parallel_wheel_y_offset_in = 0.0;

        // Body-frame X coord of the perpendicular tracking wheel's contact
        // point. Only consulted when has_perp_wheel is true.
        double perp_wheel_x_offset_in = 0.0;

        // If false, lateral body-frame displacement is taken from the
        // friction-decay + cross-coupling model (spec §8). If true, the
        // perpendicular wheel provides a direct measurement.
        bool has_perp_wheel = false;

        // Lateral friction decay rate (1/s). Used only when has_perp_wheel is
        // false; otherwise direct measurement supersedes the model. Default
        // is conservative (traction wheels — drift dies near-instantly).
        double lateral_friction_coefficient = 5.0;
    };

    explicit DeadReckoning(Config config, Pose2 initial_pose = {})
        : config_(config), pose_(initial_pose) {}

    // Feed signed wheel-rolling distances since the last update, the current
    // absolute (pre-fused) IMU heading, and the elapsed time `dt_sec` since
    // the previous update. The first call seeds the prev-heading state and
    // produces no integration step (Δθ is undefined), but the heading is
    // still recorded into pose_.
    //
    // `dt_sec` MUST be > 0; required so we can compute body-frame velocity
    // (which has units of in/s, deg/s) and so the v_y forward-Euler step is
    // well-defined when there's no perp wheel.
    void update(double parallel_wheel_delta_in,
                double perp_wheel_delta_in,
                Angle  heading_now,
                double dt_sec) {
        assert(dt_sec > 0.0 && "DeadReckoning::update requires dt_sec > 0");
        if (first_update_) {
            prev_heading_ = heading_now;
            pose_.heading = heading_now;
            first_update_ = false;
            // No Δθ baseline yet — leave velocity at its previous value
            // (initially zero). A single first-update tick can't measure a
            // rate, and zeroing here would erase any preserved-velocity state
            // a caller has set via set_pose(..., keep_velocity=true).
            return;
        }

        const double d_theta = shortest_angle(prev_heading_, heading_now).radians();

        const double d_x = parallel_wheel_delta_in
                         + d_theta * config_.parallel_wheel_y_offset_in;
        const double d_y = config_.has_perp_wheel
            ? (perp_wheel_delta_in - d_theta * config_.perp_wheel_x_offset_in)
            : 0.0;

        // Velocity tracking. v_x and ω come straight from the wheel/IMU
        // deltas. v_y either comes direct from the perp wheel (when present)
        // or is forward-Euler-integrated from the friction-decay + cross-
        // coupling model — the latter is purely an *estimate* for downstream
        // drift-aware controllers and does NOT feed back into pose
        // integration (see header comment).
        const double v_x               = d_x / dt_sec;
        const double omega_rad_per_sec = d_theta / dt_sec;
        double v_y_new                 = 0.0;
        if (config_.has_perp_wheel) {
            v_y_new = d_y / dt_sec;
        } else {
            const double v_y_dot = -config_.lateral_friction_coefficient * velocity_.v_y_ips
                                 - omega_rad_per_sec * v_x;
            v_y_new = velocity_.v_y_ips + v_y_dot * dt_sec;
        }

        const double heading_mid = prev_heading_.radians() + d_theta * 0.5;
        const double c = std::cos(heading_mid);
        const double s = std::sin(heading_mid);

        pose_.x      += c * d_x - s * d_y;
        pose_.y      += s * d_x + c * d_y;
        pose_.heading = heading_now;
        prev_heading_ = heading_now;

        velocity_.v_x_ips   = v_x;
        velocity_.v_y_ips   = v_y_new;
        velocity_.omega_dps = omega_rad_per_sec * k_rad_to_deg;
    }

    Pose2        pose()          const { return pose_; }
    BodyVelocity body_velocity() const { return velocity_; }

    // Re-localize. By default this also zeros the velocity state — the
    // expected use case is "I just placed the bot on the field and called
    // setPose at the start of autonomous; assume it isn't moving." Pass
    // keep_velocity=true to preserve the current body-frame velocity (e.g.
    // when set_pose is being used mid-motion to fuse in an absolute fix).
    //
    // Doesn't touch prev-heading: the next update() still uses the previously
    // recorded heading to compute Δθ — the integration history is independent
    // of where the pose has been teleported to.
    void set_pose(Pose2 new_pose, bool keep_velocity = false) {
        pose_ = new_pose;
        if (!keep_velocity) {
            velocity_ = BodyVelocity{};
        }
    }

    // Clear the integration history (next update() will be treated as the
    // first one). Pose stays where it is. Velocity is zeroed — without an
    // observable Δθ on the next first-update tick, retaining a stale velocity
    // would just drift the model integrator with no inputs to constrain it.
    void reset() {
        prev_heading_ = Angle{};
        first_update_ = true;
        velocity_     = BodyVelocity{};
    }

private:
    Config       config_;
    Pose2        pose_;
    BodyVelocity velocity_{};
    Angle        prev_heading_{};
    bool         first_update_ = true;
};

} // namespace pathfinder
