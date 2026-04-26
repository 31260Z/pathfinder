#pragma once

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cmath>

namespace pathfinder {

class TurnTo {
public:
    struct Options {
        Pid::Gains            heading{};
        ExitConditions::Spec  exit{};
        double                max_angular_dps = 270.0;
    };

    enum class Direction { Auto, CW, CCW };

    TurnTo(Angle target_heading, Options opts, Direction dir = Direction::Auto)
        : target_heading_(target_heading),
          opts_(opts),
          dir_(dir),
          heading_pid_(opts.heading),
          exit_(opts.exit) {}

    DriveCommand update(Pose2 current_pose, double dt_sec) {
        if (done_) {
            return DriveCommand{0.0, 0.0, true};
        }

        const double err_rad = directional_error(current_pose.heading, target_heading_, dir_);
        const double err_deg = std::abs(err_rad) * k_rad_to_deg;

        const double dt_ms = dt_sec * 1000.0;
        exit_.feed(err_deg, dt_ms);
        if (exit_.is_done()) {
            done_ = true;
            return DriveCommand{0.0, 0.0, true};
        }

        double omega_rad = heading_pid_.update(0.0, -err_rad, dt_sec);
        const double max_w_rad = opts_.max_angular_dps * k_deg_to_rad;
        if (omega_rad >  max_w_rad) omega_rad =  max_w_rad;
        if (omega_rad < -max_w_rad) omega_rad = -max_w_rad;

        return DriveCommand{0.0, omega_rad * k_rad_to_deg, false};
    }

    bool done() const { return done_; }

    void reset() {
        done_ = false;
        heading_pid_.reset();
        exit_.reset();
    }

private:
    static double directional_error(Angle from, Angle to, Direction dir) {
        const double signed_err = shortest_angle(from, to).rad;
        switch (dir) {
            case Direction::Auto: return signed_err;
            case Direction::CCW:
                return signed_err >= 0.0 ? signed_err : signed_err + k_two_pi;
            case Direction::CW:
                return signed_err <= 0.0 ? signed_err : signed_err - k_two_pi;
        }
        return signed_err;
    }

    Angle          target_heading_;
    Options        opts_;
    Direction      dir_;
    bool           done_ = false;
    Pid            heading_pid_;
    ExitConditions exit_;
};

class TurnToPoint {
public:
    using Options   = TurnTo::Options;
    using Direction = TurnTo::Direction;

    TurnToPoint(Vector2 target, Options opts, Direction dir = Direction::Auto)
        : target_(target),
          opts_(opts),
          dir_(dir),
          heading_pid_(opts.heading),
          exit_(opts.exit) {}

    DriveCommand update(Pose2 current_pose, double dt_sec) {
        if (done_) {
            return DriveCommand{0.0, 0.0, true};
        }

        const Vector2 to_target = target_ - current_pose.translation();
        if (norm(to_target) <= k_vector2_epsilon) {
            done_ = true;
            return DriveCommand{0.0, 0.0, true};
        }
        const Angle target_heading{std::atan2(to_target.y, to_target.x)};

        const double err_rad = directional_error(current_pose.heading, target_heading, dir_);
        const double err_deg = std::abs(err_rad) * k_rad_to_deg;

        exit_.feed(err_deg, dt_sec * 1000.0);
        if (exit_.is_done()) {
            done_ = true;
            return DriveCommand{0.0, 0.0, true};
        }

        double omega_rad = heading_pid_.update(0.0, -err_rad, dt_sec);
        const double max_w_rad = opts_.max_angular_dps * k_deg_to_rad;
        if (omega_rad >  max_w_rad) omega_rad =  max_w_rad;
        if (omega_rad < -max_w_rad) omega_rad = -max_w_rad;

        return DriveCommand{0.0, omega_rad * k_rad_to_deg, false};
    }

    bool done() const { return done_; }

    void reset() {
        done_ = false;
        heading_pid_.reset();
        exit_.reset();
    }

private:
    static double directional_error(Angle from, Angle to, Direction dir) {
        const double signed_err = shortest_angle(from, to).rad;
        switch (dir) {
            case Direction::Auto: return signed_err;
            case Direction::CCW:
                return signed_err >= 0.0 ? signed_err : signed_err + k_two_pi;
            case Direction::CW:
                return signed_err <= 0.0 ? signed_err : signed_err - k_two_pi;
        }
        return signed_err;
    }

    Vector2        target_;
    Options        opts_;
    Direction      dir_;
    bool           done_ = false;
    Pid            heading_pid_;
    ExitConditions exit_;
};

} // namespace pathfinder
