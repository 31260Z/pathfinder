#pragma once

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cmath>
#include <limits>

namespace pathfinder {

class MoveToPoint {
public:
    struct Options {
        Pid::Gains            along_track{};
        Pid::Gains            cross_track{};
        Pid::Gains            heading{};
        ExitConditions::Spec  exit{};
        double                max_forward_ips = 60.0;
        double                max_angular_dps = 270.0;
        bool                  reverse         = false;
    };

    MoveToPoint(Pose2 start_pose, Vector2 target, Options opts)
        : start_(start_pose),
          target_(target),
          opts_(opts),
          along_pid_(opts.along_track),
          cross_pid_(opts.cross_track),
          heading_pid_(opts.heading),
          exit_(opts.exit) {
        const Vector2 line = target - start_pose.translation();
        const double  len  = norm(line);
        if (len > k_vector2_epsilon) {
            line_dir_ = line / len;
        } else {
            // Degenerate "move" with zero length — set up to immediately exit.
            line_dir_ = Vector2{1.0, 0.0};
            degenerate_ = true;
        }
        line_heading_ = std::atan2(line_dir_.y, line_dir_.x);
    }

    DriveCommand update(Pose2 current_pose, double dt_sec) {
        if (done_) {
            return DriveCommand{0.0, 0.0, true};
        }
        if (degenerate_) {
            done_ = true;
            return DriveCommand{0.0, 0.0, true};
        }

        const Vector2 from_start  = current_pose.translation() - start_.translation();
        const double  along       = dot(from_start, line_dir_);
        // Signed perpendicular distance: positive when the bot is on the
        // +90deg-rotated side of the line direction (i.e. for a line along +X,
        // positive when bot is on +Y).
        const double  cross       = cross_signed(line_dir_, from_start);
        const double  total_len   = distance(start_.translation(), target_);
        const double  remaining   = total_len - along;
        const double  remaining_abs = std::abs(remaining);

        const double dt_ms = dt_sec * 1000.0;
        exit_.feed(remaining_abs, dt_ms);
        if (exit_.is_done()) {
            done_ = true;
            return DriveCommand{0.0, 0.0, true};
        }

        // Along-track PID outputs commanded forward velocity. Setpoint is 0
        // remaining; measurement is -remaining so that "behind target" produces
        // a positive output (drive forward).
        double v_fwd = along_pid_.update(0.0, -remaining, dt_sec);
        v_fwd = clamp_sym(v_fwd, opts_.max_forward_ips);

        // Cross-track PID converts perpendicular error into a heading offset.
        // We feed -cross so positive cross (bot on +perp side of line) yields
        // a positive PID output, which we then SUBTRACT from desired heading to
        // bias the bot back toward -perp. Clamped to +-pi/3 so it can never
        // overwhelm the primary heading regulator.
        double heading_offset = cross_pid_.update(0.0, -cross, dt_sec);
        constexpr double k_cross_offset_max = k_pi / 3.0;
        if (heading_offset >  k_cross_offset_max) heading_offset =  k_cross_offset_max;
        if (heading_offset < -k_cross_offset_max) heading_offset = -k_cross_offset_max;

        double desired_heading = line_heading_;
        if (opts_.reverse) {
            desired_heading += k_pi;
            v_fwd = -v_fwd;  // drive rear-first
        }
        desired_heading -= heading_offset;

        const double heading_err = shortest_angle(
            current_pose.heading, Angle{desired_heading}).rad;

        // Heading PID expects the error directly: setpoint=0, measurement=-err.
        double omega_rad_per_sec = heading_pid_.update(0.0, -heading_err, dt_sec);
        const double max_w_rad = opts_.max_angular_dps * k_deg_to_rad;
        omega_rad_per_sec = clamp_sym(omega_rad_per_sec, max_w_rad);

        return DriveCommand{
            v_fwd,
            omega_rad_per_sec * k_rad_to_deg,
            false,
        };
    }

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
        // perp(dir) . v  ==  dir.x * v.y - dir.y * v.x
        return dir.x * v.y - dir.y * v.x;
    }

    Pose2          start_;
    Vector2        target_;
    Options        opts_;
    Vector2        line_dir_{1.0, 0.0};
    double         line_heading_ = 0.0;
    bool           degenerate_   = false;
    bool           done_         = false;
    Pid            along_pid_;
    Pid            cross_pid_;
    Pid            heading_pid_;
    ExitConditions exit_;
};

} // namespace pathfinder
