#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {

constexpr double k_dt = 0.01;

// Drive a synthetic closed loop: at each step, advance the SimRig (which
// reads FakeMotor voltages, integrates the bot, updates FakeRotation +
// FakeImu), then call chassis.tick() (which reads sensors, integrates DR,
// writes new motor voltages). Same idea as `tests/autonomous/test_move_to`
// but routed through the full chassis pipeline.
void run_loop(Chassis& chassis, SimRig& rig, int max_steps,
              auto controller_factory) {
    auto ctrl = controller_factory();
    chassis.tick(k_dt);   // seed
    for (int i = 0; i < max_steps && !ctrl.done(); ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
        DriveCommand cmd = ctrl.update(chassis.getPose(), k_dt);
        // Voltage map mirrors what Chassis would do internally.
        const auto sides = tank_kinematics(cmd, rig.track_width_in);
        rig.left_motor->set_voltage_mv((sides.left_ips  / rig.max_forward_ips) * 12000.0);
        rig.right_motor->set_voltage_mv((sides.right_ips / rig.max_forward_ips) * 12000.0);
    }
}

MoveToPoint::Options sane_move_opts() {
    MoveToPoint::Options o{};
    o.along_track = Pid::Gains{4.0, 0.0, 0.2};
    o.cross_track = Pid::Gains{1.5, 0.0, 0.05};
    o.heading     = Pid::Gains{6.0, 0.0, 0.4};
    o.exit = ExitConditions::Spec{
        .small_error            = 0.5,
        .small_error_timeout_ms = 100.0,
        .large_error            = 2.0,
        .large_error_timeout_ms = 800.0,
        .absolute_timeout_ms    = 30000.0,
    };
    o.max_forward_ips = 60.0;
    o.max_angular_dps = 360.0;
    return o;
}

} // namespace

TEST_CASE("Chassis: closed-loop moveTo via tick() drives bot to target on +X") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    Vector2 target{24.0, 0.0};
    run_loop(chassis, rig, 4000, [&] {
        return MoveToPoint(chassis.getPose(), target, sane_move_opts());
    });

    const Pose2 p = chassis.getPose();
    CHECK(distance(p.translation(), target) < 1.5);
}

TEST_CASE("Chassis: closed-loop moveTo handles cross-track perturbation") {
    SimRig rig;
    rig.pose = Pose2{0.0, 4.0, Angle{0.0}};   // start above the line
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));
    chassis.setPose(0.0, 4.0, 0.0);

    Vector2 target{24.0, 0.0};
    run_loop(chassis, rig, 4000, [&] {
        return MoveToPoint(chassis.getPose(), target, sane_move_opts());
    });

    const Pose2 p = chassis.getPose();
    CHECK(distance(p.translation(), target) < 2.0);
}
