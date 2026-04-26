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

TEST_CASE("Chassis: constructs with FakeMotor + FakeSensors and reports origin pose") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));
    CHECK(p.heading.radians() == doctest::Approx(0.0));
}

TEST_CASE("Chassis: setPose teleports") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    chassis.setPose(24.0, -12.0, 90.0);
    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(24.0));
    CHECK(p.y == doctest::Approx(-12.0));
    CHECK(p.heading.degrees() == doctest::Approx(90.0));
}

TEST_CASE("Chassis: covariance is zero in DR") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    const Matrix3 c = chassis.getPoseCovariance();
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK(c.m[i][j] == doctest::Approx(0.0));
}

TEST_CASE("Chassis: bot/sensors/drive accessors return the configured values") {
    SimRig rig;
    rig.track_width_in = 13.5;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    CHECK(chassis.drive().track_width_in == doctest::Approx(13.5));
    CHECK(chassis.bot().footprint().length == doctest::Approx(18.0));
    CHECK(chassis.sensors().tracking_wheels().size() == 1u);
    CHECK(chassis.sensors().imus().size() == 1u);
}

TEST_CASE("Chassis: Ekf and Mcl tags construct real estimators") {
    SimRig rig;
    Chassis chassis_ekf(make_left_group(rig),
                        make_right_group(rig),
                        make_18x18_bot(),
                        make_sensors_from_rig(rig),
                        Localization::Ekf,
                        make_drive(rig));
    // EKF pose is exact at construction (state initialized to (0,0,0); the
    // 6-DOF covariance is nonzero but the mean is the seed pose).
    CHECK(chassis_ekf.getPose().x == doctest::Approx(0.0));
    CHECK(chassis_ekf.getPose().y == doctest::Approx(0.0));

    SimRig rig2;
    Chassis chassis_mcl(make_left_group(rig2),
                        make_right_group(rig2),
                        make_18x18_bot(),
                        make_sensors_from_rig(rig2),
                        Localization::Mcl{.particles = 100},
                        make_drive(rig2));
    // MCL pose is the weighted mean of particles sampled from
    // N(initial_pose, initial_xy_sigma=1.0); with 100 particles it's near
    // zero but not exactly zero. ~0.3 absolute is a generous bound.
    CHECK(std::abs(chassis_mcl.getPose().x) < 0.5);
    CHECK(std::abs(chassis_mcl.getPose().y) < 0.5);
}
