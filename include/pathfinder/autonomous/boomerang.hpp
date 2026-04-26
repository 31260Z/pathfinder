#pragma once

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

#include <cmath>

namespace pathfinder {

class Boomerang {
public:
    struct Options {
        Pid::Gains            along_track{};
        Pid::Gains            cross_track{};
        Pid::Gains            heading{};
        ExitConditions::Spec  exit{};
        double                max_forward_ips    = 60.0;
        double                max_angular_dps    = 270.0;
        double                lead               = 0.3;
        bool                  reverse            = false;
        // Drift-aware lookahead time (s). The cross-track regulator that
        // pulls the bot onto the target's heading axis uses
        // `cross + v_y · lookahead_time_sec` as the predicted error.
        // See spec §9 "Drift-aware control".
        double                lookahead_time_sec = 0.10;
        // Motion-chaining hand-off speed (see MoveToPoint::Options for the
        // full description).
        double                min_exit_speed_ips = 0.0;
    };

    Boomerang(Pose2 start_pose, Pose2 target_pose, Options opts)
        : start_(start_pose),
          target_(target_pose),
          opts_(opts),
          along_pid_(opts.along_track),
          cross_pid_(opts.cross_track),
          heading_pid_(opts.heading),
          exit_(opts.exit) {
        target_dir_ = Vector2{
            std::cos(target_pose.heading.rad),
            std::sin(target_pose.heading.rad),
        };
    }

    // Legacy / drift-unaware overload.
    DriveCommand update(Pose2 current_pose, double dt_sec) {
        return update(current_pose, BodyVelocity{}, dt_sec);
    }

    // Drift-aware update: see MoveToPoint::update for rationale.
    DriveCommand update(Pose2 current_pose, BodyVelocity body_vel, double dt_sec) {
        if (done_) {
            return DriveCommand{0.0, 0.0, true};
        }

        const double d = distance(current_pose.translation(), target_.translation());

        const double dt_ms = dt_sec * 1000.0;
        exit_.feed(d, dt_ms);
        if (exit_.is_done()) {
            done_ = true;
            if (opts_.min_exit_speed_ips > 0.0) {
                return DriveCommand{last_v_fwd_, last_omega_dps_, true};
            }
            return DriveCommand{0.0, 0.0, true};
        }

        // Carrot: project off the target along the negative-of-final-heading,
        // i.e. on the bot's approach side. Reverse flips the side so the bot
        // drives backward into the pose.
        const double sign_lead = opts_.reverse ? +1.0 : -1.0;
        const Vector2 carrot = target_.translation()
                             + (sign_lead * opts_.lead * d) * target_dir_;

        // Drive toward the carrot. The carrot-direction is the primary heading
        // target — line-following toward a moving point. Forward speed is
        // regulated on remaining distance to the *true target*, not the carrot,
        // so we decelerate as we close the pose.
        const Vector2 to_carrot = carrot - current_pose.translation();
        const double  d_carrot  = norm(to_carrot);

        double v_fwd = along_pid_.update(0.0, -d, dt_sec);
        v_fwd = clamp_sym(v_fwd, opts_.max_forward_ips);

        double carrot_heading = current_pose.heading.rad;
        if (d_carrot > k_vector2_epsilon) {
            carrot_heading = std::atan2(to_carrot.y, to_carrot.x);
        }

        // Cross-track regulation against the *target's heading axis* — the
        // perpendicular offset of the bot from the line through target_pose
        // along target_dir_. Used to nudge the final approach onto the
        // commanded heading axis. Gain should be modest; this is a fine-tune
        // on top of the carrot's primary steering.
        //
        // Drift-aware: predict the cross-track error `lookahead_time_sec`
        // ahead using the body-frame v_y (spec §9). Body-Y axis ≈ target-
        // perpendicular axis when the bot is near the target's heading;
        // adding v_y · t directly is a sound approximation for the small
        // lookahead horizons (~100 ms) we use.
        const Vector2 from_target  = current_pose.translation() - target_.translation();
        const double  cross_target = cross_signed(target_dir_, from_target);
        const double  cross_predicted = cross_target
                                      + body_vel.v_y_ips * opts_.lookahead_time_sec;
        double heading_offset = cross_pid_.update(0.0, -cross_predicted, dt_sec);
        constexpr double k_cross_offset_max = k_pi / 4.0;
        if (heading_offset >  k_cross_offset_max) heading_offset =  k_cross_offset_max;
        if (heading_offset < -k_cross_offset_max) heading_offset = -k_cross_offset_max;

        double desired_heading = carrot_heading;
        if (opts_.reverse) {
            desired_heading += k_pi;
            v_fwd = -v_fwd;
        }
        desired_heading -= heading_offset;

        const double heading_err = shortest_angle(
            current_pose.heading, Angle{desired_heading}).rad;

        double omega_rad = heading_pid_.update(0.0, -heading_err, dt_sec);
        const double max_w_rad = opts_.max_angular_dps * k_deg_to_rad;
        omega_rad = clamp_sym(omega_rad, max_w_rad);

        last_v_fwd_     = v_fwd;
        last_omega_dps_ = omega_rad * k_rad_to_deg;
        return DriveCommand{
            v_fwd,
            last_omega_dps_,
            false,
        };
    }

    void set_min_exit_speed(double ips) { opts_.min_exit_speed_ips = ips; }

    bool done() const { return done_; }

    void reset() {
        done_ = false;
        along_pid_.reset();
        cross_pid_.reset();
        heading_pid_.reset();
        exit_.reset();
    }

private:
    static double clamp_sym(double v, double mag) {
        if (mag <= 0.0) return 0.0;
        if (v >  mag) return  mag;
        if (v < -mag) return -mag;
        return v;
    }

    static double cross_signed(Vector2 dir, Vector2 v) {
        return dir.x * v.y - dir.y * v.x;
    }

    Pose2          start_;
    Pose2          target_;
    Options        opts_;
    Vector2        target_dir_{1.0, 0.0};
    bool           done_           = false;
    double         last_v_fwd_     = 0.0;
    double         last_omega_dps_ = 0.0;
    Pid            along_pid_;
    Pid            cross_pid_;
    Pid            heading_pid_;
    ExitConditions exit_;
};

} // namespace pathfinder
