#pragma once

#include <pathfinder/autonomous/catmull_rom.hpp>
#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <optional>
#include <utility>
#include <vector>

namespace pathfinder {

struct Waypoint {
    double                position_x = 0.0;
    double                position_y = 0.0;
    std::optional<double> speed_cap_ips      = std::nullopt;
    std::optional<double> heading_deg        = std::nullopt;
    std::function<void()> on_arrive          = {};
    bool                  must_pass_through  = true;

    Vector2 point() const { return {position_x, position_y}; }
};

class PurePursuit {
public:
    struct Options {
        Pid::Gains            forward{};
        Pid::Gains            heading{};
        ExitConditions::Spec  exit{};
        double                lookahead_in    = 12.0;
        double                max_forward_ips = 60.0;
        double                max_angular_dps = 270.0;
        bool                  reverse         = false;
    };

    PurePursuit(catmull_rom::Path path, Options opts)
        : path_(std::move(path)),
          opts_(opts),
          forward_pid_(opts.forward),
          heading_pid_(opts.heading),
          exit_(opts.exit) {
        waypoint_callbacks_fired_.assign(path_.waypoint_count(), false);
        waypoint_speed_caps_.assign(path_.waypoint_count(), std::nullopt);
    }

    PurePursuit(catmull_rom::Path path, Options opts, std::vector<Waypoint> waypoint_meta)
        : path_(std::move(path)),
          opts_(opts),
          waypoint_meta_(std::move(waypoint_meta)),
          forward_pid_(opts.forward),
          heading_pid_(opts.heading),
          exit_(opts.exit) {
        const std::size_t n = path_.waypoint_count();
        waypoint_callbacks_fired_.assign(n, false);
        waypoint_speed_caps_.assign(n, std::nullopt);
        for (std::size_t i = 0; i < waypoint_meta_.size() && i < n; ++i) {
            waypoint_speed_caps_[i] = waypoint_meta_[i].speed_cap_ips;
        }
    }

    DriveCommand update(Pose2 current_pose, double dt_sec) {
        if (done_) {
            return DriveCommand{0.0, 0.0, true};
        }

        double s_now = 0.0;
        path_.closest_point_arclength(current_pose.translation(), s_now);
        if (s_now > progress_s_) progress_s_ = s_now;

        // Fire on_arrive callbacks for any waypoints we have just passed.
        for (std::size_t i = 0; i < path_.waypoint_count(); ++i) {
            if (waypoint_callbacks_fired_[i]) continue;
            if (s_now >= path_.waypoint_arclength(i)) {
                waypoint_callbacks_fired_[i] = true;
                if (i < waypoint_meta_.size() && waypoint_meta_[i].on_arrive) {
                    waypoint_meta_[i].on_arrive();
                }
            }
        }

        const double total_len   = path_.total_length_in();
        const double remaining   = total_len - s_now;
        const Vector2 endpoint   = path_.sample_at_arclength(total_len);
        const double  d_endpoint = distance(current_pose.translation(), endpoint);

        const double dt_ms = dt_sec * 1000.0;
        exit_.feed(d_endpoint, dt_ms);
        if (exit_.is_done()) {
            done_ = true;
            // Fire any remaining waypoint callbacks the projection didn't trigger
            // (typical when the bot stops just short of the endpoint's arclength).
            for (std::size_t i = 0; i < path_.waypoint_count(); ++i) {
                if (waypoint_callbacks_fired_[i]) continue;
                waypoint_callbacks_fired_[i] = true;
                if (i < waypoint_meta_.size() && waypoint_meta_[i].on_arrive) {
                    waypoint_meta_[i].on_arrive();
                }
            }
            return DriveCommand{0.0, 0.0, true};
        }

        // Lookahead point: clamp to end of path if past it.
        const double s_look = std::min(s_now + opts_.lookahead_in, total_len);
        const Vector2 lookahead_pt = path_.sample_at_arclength(s_look);

        // Heading toward lookahead.
        const Vector2 to_la = lookahead_pt - current_pose.translation();
        double desired_heading = current_pose.heading.rad;
        if (norm(to_la) > k_vector2_epsilon) {
            desired_heading = std::atan2(to_la.y, to_la.x);
        }
        if (opts_.reverse) {
            desired_heading += k_pi;
        }

        const double heading_err = shortest_angle(
            current_pose.heading, Angle{desired_heading}).rad;

        // Forward PID on remaining path length: we want remaining -> 0.
        double v_fwd = forward_pid_.update(0.0, -remaining, dt_sec);

        // Active speed cap from the nearest "upcoming" waypoint.
        const double cap = active_speed_cap(s_now);
        const double v_max = std::min(opts_.max_forward_ips, cap);
        v_fwd = clamp_sym(v_fwd, v_max);
        if (opts_.reverse) v_fwd = -v_fwd;

        double omega_rad = heading_pid_.update(0.0, -heading_err, dt_sec);
        const double max_w_rad = opts_.max_angular_dps * k_deg_to_rad;
        omega_rad = clamp_sym(omega_rad, max_w_rad);

        return DriveCommand{
            v_fwd,
            omega_rad * k_rad_to_deg,
            false,
        };
    }

    bool   done() const { return done_; }
    double progress_distance_in() const { return progress_s_; }

    void reset() {
        done_       = false;
        progress_s_ = 0.0;
        forward_pid_.reset();
        heading_pid_.reset();
        exit_.reset();
        waypoint_callbacks_fired_.assign(path_.waypoint_count(), false);
    }

private:
    static double clamp_sym(double v, double mag) {
        if (mag <= 0.0) return 0.0;
        if (v >  mag) return  mag;
        if (v < -mag) return -mag;
        return v;
    }

    double active_speed_cap(double s_now) const {
        // Use the cap of the next upcoming waypoint (strictly ahead of s_now).
        // If we are past the last waypoint, fall back to its cap. Step-wise;
        // a smarter pass could blend caps across segments.
        double cap = std::numeric_limits<double>::infinity();
        const std::size_t n = path_.waypoint_count();
        for (std::size_t i = 0; i < n; ++i) {
            if (path_.waypoint_arclength(i) > s_now + 1e-9) {
                if (waypoint_speed_caps_[i].has_value()) {
                    cap = *waypoint_speed_caps_[i];
                }
                return cap;
            }
        }
        if (n > 0 && waypoint_speed_caps_[n - 1].has_value()) {
            cap = *waypoint_speed_caps_[n - 1];
        }
        return cap;
    }

    catmull_rom::Path                      path_;
    Options                                opts_;
    std::vector<Waypoint>                  waypoint_meta_{};
    std::vector<std::optional<double>>     waypoint_speed_caps_{};
    std::vector<bool>                      waypoint_callbacks_fired_{};
    double                                 progress_s_ = 0.0;
    bool                                   done_       = false;
    Pid                                    forward_pid_;
    Pid                                    heading_pid_;
    ExitConditions                         exit_;
};

} // namespace pathfinder
