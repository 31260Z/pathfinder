#pragma once

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>

#include <cmath>

namespace pathfinder_test {

// Minimal kinematic simulator: integrates a DriveCommand into a Pose2 as a
// perfect double-integrator (no slip, no lag, no inertia). Mid-heading
// integration scheme reduces curvature error on tight turns.
struct SimBot {
    pathfinder::Pose2 pose{};

    pathfinder::Pose2 step(pathfinder::DriveCommand cmd, double dt_sec) {
        const double omega_rad_per_sec = cmd.angular_velocity_dps * pathfinder::k_deg_to_rad;
        const double new_h = pose.heading.rad + omega_rad_per_sec * dt_sec;
        const double mid_h = 0.5 * (pose.heading.rad + new_h);
        pose.x += cmd.forward_velocity_ips * std::cos(mid_h) * dt_sec;
        pose.y += cmd.forward_velocity_ips * std::sin(mid_h) * dt_sec;
        pose.heading = pathfinder::Angle{new_h};
        return pose;
    }
};

} // namespace pathfinder_test
