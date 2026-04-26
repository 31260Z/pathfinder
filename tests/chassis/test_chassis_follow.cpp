#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

#include <vector>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {

constexpr double k_dt = 0.01;

PurePursuit::Options sane_pp_opts() {
    PurePursuit::Options o{};
    o.forward = Pid::Gains{4.0, 0.0, 0.2};
    o.heading = Pid::Gains{6.0, 0.0, 0.4};
    o.exit = ExitConditions::Spec{
        .small_error            = 0.7,
        .small_error_timeout_ms = 100.0,
        .large_error            = 2.0,
        .large_error_timeout_ms = 800.0,
        .absolute_timeout_ms    = 60000.0,
    };
    o.lookahead_in    = 8.0;
    o.max_forward_ips = 50.0;
    o.max_angular_dps = 360.0;
    return o;
}

} // namespace

TEST_CASE("Chassis: closed-loop follow tracks a multi-waypoint path") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    std::vector<Vector2> wpts{
        {0, 0}, {12, 4}, {24, -4}, {36, 0}, {48, 4},
    };
    catmull_rom::Path path(wpts);
    PurePursuit ctrl(std::move(path), sane_pp_opts());

    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 8000 && !ctrl.done(); ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
        DriveCommand cmd = ctrl.update(chassis.getPose(), k_dt);
        const auto sides = tank_kinematics(cmd, rig.track_width_in);
        rig.left_motor->set_voltage_mv((sides.left_ips  / rig.max_forward_ips) * 12000.0);
        rig.right_motor->set_voltage_mv((sides.right_ips / rig.max_forward_ips) * 12000.0);
    }

    const Pose2 p = chassis.getPose();
    CHECK(distance(p.translation(), wpts.back()) < 2.0);
}

TEST_CASE("Chassis::follow: vector<Waypoint> overload throws on too-few waypoints") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    std::vector<Waypoint> short_path{Waypoint{0, 0}};
    CHECK_THROWS_AS(chassis.follow(short_path), std::invalid_argument);
}

TEST_CASE("Chassis::follow: vector<Vector2> overload throws on too-few waypoints") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    std::vector<Vector2> short_path{{0, 0}};
    CHECK_THROWS_AS(chassis.follow(short_path), std::invalid_argument);
}
