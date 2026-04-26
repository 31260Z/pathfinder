#pragma once

#include <pathfinder/geometry/angle.hpp>

namespace pathfinder {

struct DriveCommand {
    double forward_velocity_ips = 0.0;
    double angular_velocity_dps = 0.0;
    bool   done                 = false;
};

struct PerSideVelocities {
    double left_ips  = 0.0;
    double right_ips = 0.0;
};

// Differential-drive inverse kinematics: split a (v, omega) command into
// per-side wheel linear velocities given track width.
//
// Sign convention: positive omega_dps is heading-rate in our convention
// (+X -> +Y), which puts the rotation center to the bot's right side and
// makes the LEFT wheel travel the longer arc, hence faster.
//
//     v_left  = v + omega_rad * (W / 2)
//     v_right = v - omega_rad * (W / 2)
inline PerSideVelocities tank_kinematics(DriveCommand cmd, double track_width_in) {
    const double omega_rad = cmd.angular_velocity_dps * k_deg_to_rad;
    const double half_w    = 0.5 * track_width_in;
    return PerSideVelocities{
        cmd.forward_velocity_ips + omega_rad * half_w,
        cmd.forward_velocity_ips - omega_rad * half_w,
    };
}

} // namespace pathfinder
