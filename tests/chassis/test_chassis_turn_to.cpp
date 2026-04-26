#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

#include <cmath>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {

constexpr double k_dt = 0.01;

TurnTo::Options sane_turn_opts() {
    TurnTo::Options o{};
    o.heading = Pid::Gains{6.0, 0.0, 0.4};
    o.exit = ExitConditions::Spec{
        .small_error            = 1.0,
        .small_error_timeout_ms = 100.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 500.0,
        .absolute_timeout_ms    = 10000.0,
    };
    o.max_angular_dps = 360.0;
    return o;
}

} // namespace

TEST_CASE("Chassis: closed-loop turnTo rotates to target heading") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    TurnTo ctrl(Angle::degrees(90.0), sane_turn_opts());

    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 4000 && !ctrl.done(); ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
        DriveCommand cmd = ctrl.update(chassis.getPose(), k_dt);
        const auto sides = tank_kinematics(cmd, rig.track_width_in);
        rig.left_motor->set_voltage_mv((sides.left_ips  / rig.max_forward_ips) * 12000.0);
        rig.right_motor->set_voltage_mv((sides.right_ips / rig.max_forward_ips) * 12000.0);
    }

    const Pose2 p = chassis.getPose();
    const double err = std::abs(shortest_angle(p.heading, Angle::degrees(90.0)).rad);
    CHECK(err < 0.1);   // ~5.7 deg
    CHECK(p.x == doctest::Approx(0.0).epsilon(0.1));
    CHECK(p.y == doctest::Approx(0.0).epsilon(0.1));
}

TEST_CASE("Chassis: turnTo a point computes correct facing direction") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    Vector2 target{12.0, 12.0};   // facing this from origin = 45deg
    TurnToPoint ctrl(target, sane_turn_opts());

    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 4000 && !ctrl.done(); ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
        DriveCommand cmd = ctrl.update(chassis.getPose(), k_dt);
        const auto sides = tank_kinematics(cmd, rig.track_width_in);
        rig.left_motor->set_voltage_mv((sides.left_ips  / rig.max_forward_ips) * 12000.0);
        rig.right_motor->set_voltage_mv((sides.right_ips / rig.max_forward_ips) * 12000.0);
    }

    const Pose2 p = chassis.getPose();
    const double err = std::abs(shortest_angle(p.heading, Angle::degrees(45.0)).rad);
    CHECK(err < 0.1);
}
