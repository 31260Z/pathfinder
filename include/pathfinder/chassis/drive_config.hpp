#pragma once

#include <pathfinder/geometry/angle.hpp>

#include <cmath>

namespace pathfinder {

// Kinematic + electrical parameters of the drivetrain. The chassis layer uses
// these to convert (a) `DriveCommand{forward_velocity_ips, angular_velocity_dps}`
// to per-side velocities via `tank_kinematics`, then (b) per-side ips into a
// motor voltage command.
//
// Defaults match the spec quick-start (§14): an 18×18 bot with 12" track
// width, 3.25" omni wheels, 200rpm internal cartridge driving through a
// 36:84 external gear (wheel turns 36/84 of a motor revolution). Override
// any field via designated initializer at construction time.
struct Drive {
    // ── Geometry ─────────────────────────────────────────────────────────
    double track_width_in    = 12.0;
    double wheel_diameter_in = 3.25;
    double gear_ratio        = 36.0 / 84.0;   // wheel revs per motor rev

    // ── Drift modelling (consumed by the EKF + drift-aware regulator in
    //    Wave D; defaulted conservatively for traction wheels) ───────────
    double lateral_friction_coefficient = 5.0;   // 1/s

    // ── Per-call default speed limits ────────────────────────────────────
    double max_forward_ips      = 60.0;
    double max_angular_dps      = 270.0;
    double max_forward_voltage_mv = 12000.0;

    // ── Voltage mapping ──────────────────────────────────────────────────
    // If non-zero: `voltage_mv = voltage_per_ips · ips`, clamped to ±12000.
    // If zero (default): use a proportional bang-bang map
    // `voltage_mv = (ips / max_forward_ips) · max_forward_voltage_mv`.
    double voltage_per_ips = 0.0;

    // Convert a wheel linear velocity (in/s) into a motor-shaft RPM. Useful
    // for callers that want to drive PROS' velocity control instead of the
    // raw voltage path. Math: wheel_ips → wheel_rps = ips / (π · d) →
    // motor_rps = wheel_rps / gear_ratio → motor_rpm = ·60.
    double ips_to_motor_rpm(double ips) const {
        const double circumference = k_pi * wheel_diameter_in;
        if (circumference <= 0.0 || gear_ratio <= 0.0) return 0.0;
        const double wheel_rps = ips / circumference;
        const double motor_rps = wheel_rps / gear_ratio;
        return motor_rps * 60.0;
    }

    // Convert a per-side wheel velocity command (in/s) into a motor voltage
    // (mV), respecting the `voltage_per_ips` override (when non-zero) and
    // the proportional fallback (when zero). Output clamped to ±12000.
    double ips_to_voltage_mv(double ips) const {
        double mv = 0.0;
        if (voltage_per_ips > 0.0) {
            mv = voltage_per_ips * ips;
        } else if (max_forward_ips > 0.0) {
            mv = (ips / max_forward_ips) * max_forward_voltage_mv;
        }
        if (mv >  12000.0) mv =  12000.0;
        if (mv < -12000.0) mv = -12000.0;
        return mv;
    }
};

} // namespace pathfinder
