#include <doctest/doctest.h>

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/geometry/angle.hpp>

using pathfinder::DriveCommand;
using pathfinder::PerSideVelocities;
using pathfinder::tank_kinematics;
using pathfinder::k_deg_to_rad;

TEST_CASE("Pure forward: both sides equal to forward velocity") {
    DriveCommand cmd{20.0, 0.0, false};
    PerSideVelocities v = tank_kinematics(cmd, 12.0);
    CHECK(v.left_ips  == doctest::Approx(20.0));
    CHECK(v.right_ips == doctest::Approx(20.0));
}

TEST_CASE("Pure backward: both sides equal to negative") {
    DriveCommand cmd{-15.0, 0.0, false};
    PerSideVelocities v = tank_kinematics(cmd, 12.0);
    CHECK(v.left_ips  == doctest::Approx(-15.0));
    CHECK(v.right_ips == doctest::Approx(-15.0));
}

TEST_CASE("Pure rotation positive omega: left faster than right") {
    DriveCommand cmd{0.0, 90.0, false};  // 90 deg/s = pi/2 rad/s
    PerSideVelocities v = tank_kinematics(cmd, 12.0);
    const double half_w_omega = (90.0 * k_deg_to_rad) * 6.0;  // 6 in half-track
    CHECK(v.left_ips  == doctest::Approx( half_w_omega));
    CHECK(v.right_ips == doctest::Approx(-half_w_omega));
    CHECK(v.left_ips  > v.right_ips);
}

TEST_CASE("Pure rotation negative omega: right faster than left") {
    DriveCommand cmd{0.0, -45.0, false};
    PerSideVelocities v = tank_kinematics(cmd, 10.0);
    CHECK(v.left_ips  < v.right_ips);
    CHECK(v.left_ips  == doctest::Approx(-(45.0 * k_deg_to_rad) * 5.0));
    CHECK(v.right_ips == doctest::Approx( (45.0 * k_deg_to_rad) * 5.0));
}

TEST_CASE("Mixed forward + rotation: each side is sum of components") {
    DriveCommand cmd{30.0, 60.0, false};
    PerSideVelocities v = tank_kinematics(cmd, 14.0);
    const double half_w_omega = (60.0 * k_deg_to_rad) * 7.0;
    CHECK(v.left_ips  == doctest::Approx(30.0 + half_w_omega));
    CHECK(v.right_ips == doctest::Approx(30.0 - half_w_omega));
}

TEST_CASE("Zero command: zero velocities") {
    DriveCommand cmd{0.0, 0.0, true};
    PerSideVelocities v = tank_kinematics(cmd, 12.0);
    CHECK(v.left_ips  == doctest::Approx(0.0));
    CHECK(v.right_ips == doctest::Approx(0.0));
}
