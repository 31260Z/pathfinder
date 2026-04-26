#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cmath>

namespace pathfinder {

// Tier-1 localization: integrates wheel-encoder displacements + an absolute
// IMU heading into a Pose2. Sensor-agnostic — accepts numbers (signed inches
// of wheel rolling distance, radians of heading), so this module is host-side
// testable and pulls in no PROS headers.
//
// Math (per spec App. A):
//   For each step, the wheel center at body-frame (x_p, y_p) on a body whose
//   center translates by (d_x, d_y) and rotates by Δθ moves by
//     (d_x − Δθ · y_p,  d_y + Δθ · x_q)
//   in body frame. The parallel wheel reads body-X, the perp wheel reads
//   body-Y. Solving:
//     d_x = Δs_par  + Δθ · y_p
//     d_y = Δs_perp − Δθ · x_q       (or 0 if no perp wheel)
//   The body-frame displacement is rotated into world frame using the heading
//   at the midpoint of the step (better than start- or end-heading for
//   integrating curved motion — second-order accurate Heun-like step).
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

        // If false, lateral body-frame displacement is taken as zero (no slip
        // assumption — appropriate for tank drives without a perp pod).
        bool has_perp_wheel = false;
    };

    explicit DeadReckoning(Config config, Pose2 initial_pose = {})
        : config_(config), pose_(initial_pose) {}

    // Feed signed wheel-rolling distances since the last update along with
    // the current absolute (pre-fused) IMU heading. The first call seeds the
    // prev-heading state and produces no integration step (Δθ is undefined),
    // but the heading is still recorded into pose_.
    void update(double parallel_wheel_delta_in,
                double perp_wheel_delta_in,
                Angle  heading_now) {
        if (first_update_) {
            prev_heading_ = heading_now;
            pose_.heading = heading_now;
            first_update_ = false;
            return;
        }

        const double d_theta = shortest_angle(prev_heading_, heading_now).radians();

        const double d_x = parallel_wheel_delta_in
                         + d_theta * config_.parallel_wheel_y_offset_in;
        const double d_y = config_.has_perp_wheel
            ? (perp_wheel_delta_in - d_theta * config_.perp_wheel_x_offset_in)
            : 0.0;

        const double heading_mid = prev_heading_.radians() + d_theta * 0.5;
        const double c = std::cos(heading_mid);
        const double s = std::sin(heading_mid);

        pose_.x      += c * d_x - s * d_y;
        pose_.y      += s * d_x + c * d_y;
        pose_.heading = heading_now;
        prev_heading_ = heading_now;
    }

    Pose2 pose() const { return pose_; }

    // Re-localize. Doesn't touch prev-heading: the next update() still uses
    // the previously recorded heading to compute Δθ — the integration history
    // is independent of where the pose has been teleported to.
    void set_pose(Pose2 new_pose) { pose_ = new_pose; }

    // Clear the integration history (next update() will be treated as the
    // first one). Pose stays where it is.
    void reset() {
        prev_heading_ = Angle{};
        first_update_ = true;
    }

private:
    Config config_;
    Pose2  pose_;
    Angle  prev_heading_{};
    bool   first_update_ = true;
};

} // namespace pathfinder
