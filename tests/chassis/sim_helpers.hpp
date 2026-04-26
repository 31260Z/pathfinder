#pragma once

// Test-side helpers for wiring `FakeMotor` voltage commands back into a
// kinematic SimBot whose pose drives `FakeRotation` + `FakeImu` readings.
// The chassis polls the same FakeRotation / FakeImu instances through its
// real `IRotation` / `IImu` interfaces — so calling `chassis.tick(dt)` after
// `update_simbot_from_motors(...)` produces the closed-loop behaviour that
// the chassis tests need.

#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/driving/mocks.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/sensors/mocks.hpp>

#include <cmath>
#include <memory>

namespace pathfinder_test {

// Linear voltage→velocity model: full-scale (±12000 mV) maps to
// `max_forward_ips` of wheel velocity. Matches the chassis's default
// proportional voltage-mapping path (Drive::voltage_per_ips == 0).
inline double voltage_to_ips(double mv, double max_forward_ips) {
    return (mv / 12000.0) * max_forward_ips;
}

struct SimRig {
    std::shared_ptr<pathfinder::FakeMotor>    left_motor   = std::make_shared<pathfinder::FakeMotor>();
    std::shared_ptr<pathfinder::FakeMotor>    right_motor  = std::make_shared<pathfinder::FakeMotor>();
    std::shared_ptr<pathfinder::FakeRotation> par_wheel    = std::make_shared<pathfinder::FakeRotation>();
    std::shared_ptr<pathfinder::FakeImu>      imu          = std::make_shared<pathfinder::FakeImu>();

    pathfinder::Pose2 pose{};
    double            track_width_in    = 12.0;
    double            wheel_diameter_in = 3.25;
    double            max_forward_ips   = 60.0;

    // Advance the simulated bot one step. Reads the latest commanded voltages
    // from the FakeMotors, converts to per-side ips, integrates a midpoint
    // step, and updates the FakeRotation (forward distance / wheel
    // circumference) and FakeImu (heading) so the chassis sees consistent
    // sensor data on the next `tick(dt)`.
    void step(double dt) {
        const double l_ips = voltage_to_ips(left_motor->last_voltage_mv(),  max_forward_ips);
        const double r_ips = voltage_to_ips(right_motor->last_voltage_mv(), max_forward_ips);
        // tank_kinematics: v_left = v + omega·W/2, v_right = v - omega·W/2,
        // so to recover (v, omega) from per-side ips:
        //   v     = (v_left + v_right) / 2
        //   omega = (v_left - v_right) / W            (rad/s)
        const double v     = 0.5 * (l_ips + r_ips);
        const double omega = (l_ips - r_ips) / track_width_in;

        const double new_h = pose.heading.rad + omega * dt;
        const double mid_h = 0.5 * (pose.heading.rad + new_h);
        pose.x += v * std::cos(mid_h) * dt;
        pose.y += v * std::sin(mid_h) * dt;
        pose.heading = pathfinder::Angle{new_h};

        // Push the chassis-visible sensor values: the parallel wheel measures
        // the forward distance the bot has driven along its body-X axis (=
        // ∫v dt for a chassis-centerline pod). Convert distance → revs via
        // π·d.
        const double circumference = pathfinder::k_pi * wheel_diameter_in;
        if (circumference > 0.0) {
            par_wheel->set_position(par_wheel->position_revolutions()
                                    + (v * dt) / circumference);
        }
        imu->set_heading(pose.heading);
    }
};

inline pathfinder::Sensors make_sensors_from_rig(const SimRig& rig) {
    pathfinder::Sensors s;
    pathfinder::TrackingWheel par{};
    par.sensor             = rig.par_wheel;
    par.wheel              = pathfinder::Wheel::Custom;
    par.custom_diameter_in = rig.wheel_diameter_in;
    par.axis               = pathfinder::Axis::X;
    par.offset             = {9.0, 9.0};   // chassis center for an 18×18 BackLeft bot
    s.add(par);

    pathfinder::Imu imu_cfg{};
    imu_cfg.sensor       = rig.imu;
    imu_cfg.mounting     = pathfinder::ImuMounting::ZDown_XForward;   // sign +1 — direct
    imu_cfg.offset_xy_in = {9.0, 9.0};
    s.add(imu_cfg);
    return s;
}

inline pathfinder::Bot make_18x18_bot() {
    pathfinder::Bot b;
    b.footprint(18.0, 18.0).origin(pathfinder::Corner::BackLeft);
    return b;
}

inline pathfinder::Drive make_drive(const SimRig& rig) {
    pathfinder::Drive d{};
    d.track_width_in    = rig.track_width_in;
    d.wheel_diameter_in = rig.wheel_diameter_in;
    d.max_forward_ips   = rig.max_forward_ips;
    return d;
}

inline pathfinder::MotorGroup make_left_group(const SimRig& rig) {
    return pathfinder::MotorGroup{rig.left_motor};
}

inline pathfinder::MotorGroup make_right_group(const SimRig& rig) {
    return pathfinder::MotorGroup{rig.right_motor};
}

} // namespace pathfinder_test
