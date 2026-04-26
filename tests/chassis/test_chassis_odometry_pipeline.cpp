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
}

TEST_CASE("Chassis: ticking with no commanded motion holds pose at origin") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    for (int i = 0; i < 50; ++i) chassis.tick(k_dt);
    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(p.y == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(p.heading.radians() == doctest::Approx(0.0));
}

TEST_CASE("Chassis: forward encoder ticks integrate to +X with heading 0") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    // Seed: first tick records the encoder baseline.
    chassis.tick(k_dt);

    // Drive 12" forward by advancing the parallel-wheel encoder by 12 / (π·d) revs.
    const double circumference = k_pi * rig.wheel_diameter_in;
    const double delta_revs    = 12.0 / circumference;
    rig.par_wheel->set_position(rig.par_wheel->position_revolutions() + delta_revs);

    chassis.tick(k_dt);
    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(12.0).epsilon(1e-9));
    CHECK(p.y == doctest::Approx(0.0).epsilon(1e-9));
}

TEST_CASE("Chassis: IMU heading change with no encoder change rotates pose only") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    chassis.tick(k_dt);
    rig.imu->set_heading(Angle::degrees(45.0));
    chassis.tick(k_dt);

    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(p.y == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(p.heading.degrees() == doctest::Approx(45.0));
}

TEST_CASE("Chassis: forward at heading 90deg integrates to +Y") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    chassis.tick(k_dt);                       // seed
    rig.imu->set_heading(Angle::degrees(90.0));
    chassis.tick(k_dt);                       // pure rotation, mid_h = 45deg

    // Now drive forward 6" with heading already at 90 — both before/after
    // headings are 90deg so the mid-heading is also 90, integrating onto +Y.
    const double circumference = k_pi * rig.wheel_diameter_in;
    rig.par_wheel->set_position(rig.par_wheel->position_revolutions()
                                + 6.0 / circumference);
    chassis.tick(k_dt);

    const Pose2 p = chassis.getPose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(p.y == doctest::Approx(6.0).epsilon(1e-6));
    CHECK(p.heading.degrees() == doctest::Approx(90.0));
}

TEST_CASE("Chassis: SimRig step() drives forward and chassis tracks it") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    // Set both motors to full forward via the FakeMotor's last_voltage_mv —
    // the SimRig integrator picks them up from there.
    rig.left_motor->set_voltage_mv(6000.0);
    rig.right_motor->set_voltage_mv(6000.0);

    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 100; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    const Pose2 p = chassis.getPose();
    // 6000 mV → 30 ips × 1.0 s = 30 in.
    CHECK(p.x == doctest::Approx(30.0).epsilon(1e-3));
    CHECK(p.y == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(p.heading.degrees() == doctest::Approx(0.0));
}
