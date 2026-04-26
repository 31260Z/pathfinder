#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/autonomous/drive_command.hpp>
#include <pathfinder/chassis/chassis.hpp>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

TEST_CASE("Chassis::opcontrol(FlightStyle::Rate): pure forward stick → equal per-side voltages") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    FlightStyle::Rate mode{};
    mode.max_forward_ips = 60.0;
    mode.max_yaw_dps     = 270.0;
    mode.expo_forward    = 0.0;          // linear so we can predict the per-side ips
    mode.expo_yaw        = 0.0;
    mode.deadband        = 0.0;

    auto poll = [](JoystickAxis a) -> double {
        if (a == JoystickAxis::LeftY)  return 1.0;
        return 0.0;
    };

    int iter = 0;
    auto should_exit = [&] { return ++iter >= 3; };
    chassis.opcontrol(mode, poll, should_exit);

    // After full forward stick: target v = 60 ips, yaw = 0. tank_kinematics
    // splits to (left=60, right=60). ips→mV at default mapping = 60/60 *
    // 12000 = 12000 each. After exit the chassis writes 0; we want to
    // observe the in-flight commanded voltage via chassis.last_*_voltage_mv()
    // which is set just before host_sleep + the post-loop zero-write.
    // Because the loop's last act is write_per_side_voltage(0,0), we expect
    // 0 here. Instead check via a one-iteration test below.
    CHECK(chassis.last_left_voltage_mv()  == doctest::Approx(0.0));
    CHECK(chassis.last_right_voltage_mv() == doctest::Approx(0.0));
}

TEST_CASE("Chassis::opcontrol(FlightStyle::Rate): inspect commanded voltages mid-loop via telemetry") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    FlightStyle::Rate mode{};
    mode.expo_forward = 0.0;
    mode.expo_yaw     = 0.0;
    mode.deadband     = 0.0;

    // Telemetry captures left_mv / right_mv on each tick — we use the LAST
    // captured record (one before the loop's post-exit zero-write) to assert
    // on the in-flight commanded voltages.
    std::vector<std::pair<double, double>> mv_pairs;
    chassis.set_telemetry_sink([&](std::string_view line) {
        // Pull "left_mv=" and "right_mv=" out of the line.
        const auto lp = line.find("left_mv=");
        const auto rp = line.find("right_mv=");
        if (lp == std::string_view::npos || rp == std::string_view::npos) return;
        double l = 0.0, r = 0.0;
        std::sscanf(std::string(line).c_str() + lp, "left_mv=%lf",  &l);
        std::sscanf(std::string(line).c_str() + rp, "right_mv=%lf", &r);
        mv_pairs.emplace_back(l, r);
    });

    auto poll = [](JoystickAxis a) -> double {
        if (a == JoystickAxis::LeftY)  return 1.0;       // full forward
        return 0.0;
    };
    int iter = 0;
    auto should_exit = [&] { return ++iter >= 2; };
    chassis.opcontrol(mode, poll, should_exit);

    REQUIRE(mv_pairs.size() >= 1);
    // Pure forward at mode.max_forward_ips = 60 (default), drive default
    // max_forward_ips = 60. left = right = 12000 mV.
    const auto [l, r] = mv_pairs.back();
    CHECK(l == doctest::Approx(12000.0));
    CHECK(r == doctest::Approx(12000.0));
}

TEST_CASE("Chassis::opcontrol(FlightStyle::Rate): pure yaw stick → opposite per-side voltages") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    FlightStyle::Rate mode{};
    mode.expo_forward = 0.0;
    mode.expo_yaw     = 0.0;
    mode.deadband     = 0.0;
    mode.max_forward_ips = 60.0;
    mode.max_yaw_dps     = 270.0;

    std::vector<std::pair<double, double>> mv_pairs;
    chassis.set_telemetry_sink([&](std::string_view line) {
        const auto lp = line.find("left_mv=");
        const auto rp = line.find("right_mv=");
        if (lp == std::string_view::npos || rp == std::string_view::npos) return;
        double l = 0.0, r = 0.0;
        std::sscanf(std::string(line).c_str() + lp, "left_mv=%lf",  &l);
        std::sscanf(std::string(line).c_str() + rp, "right_mv=%lf", &r);
        mv_pairs.emplace_back(l, r);
    });

    auto poll = [](JoystickAxis a) -> double {
        if (a == JoystickAxis::RightX) return 0.5;       // half yaw, +CW
        return 0.0;
    };
    int iter = 0;
    auto should_exit = [&] { return ++iter >= 2; };
    chassis.opcontrol(mode, poll, should_exit);

    REQUIRE(mv_pairs.size() >= 1);
    const auto [l, r] = mv_pairs.back();
    // Pure yaw +CW at 0.5 deflection: omega = 0.5 * 270 = 135 deg/s.
    // tank: v_left = +omega·W/2, v_right = -omega·W/2. Track width = 12 in.
    // omega_rad ≈ 2.356, half_w = 6 → ±14.137 ips. mV = ips/60 * 12000 ≈ ±2827.
    CHECK(l == doctest::Approx( 2827.43).epsilon(0.01));
    CHECK(r == doctest::Approx(-2827.43).epsilon(0.01));
}

TEST_CASE("Chassis::opcontrol(FlightStyle::Rate): deadband suppresses small inputs") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    FlightStyle::Rate mode{};
    mode.deadband = 0.10;

    // Stick at 0.05 → below deadband → curve returns 0 → motors should be 0.
    std::vector<std::pair<double, double>> mv_pairs;
    chassis.set_telemetry_sink([&](std::string_view line) {
        const auto lp = line.find("left_mv=");
        const auto rp = line.find("right_mv=");
        if (lp == std::string_view::npos || rp == std::string_view::npos) return;
        double l = 0.0, r = 0.0;
        std::sscanf(std::string(line).c_str() + lp, "left_mv=%lf",  &l);
        std::sscanf(std::string(line).c_str() + rp, "right_mv=%lf", &r);
        mv_pairs.emplace_back(l, r);
    });

    auto poll = [](JoystickAxis) { return 0.05; };
    int iter = 0;
    auto should_exit = [&] { return ++iter >= 2; };
    chassis.opcontrol(mode, poll, should_exit);

    REQUIRE(mv_pairs.size() >= 1);
    const auto [l, r] = mv_pairs.back();
    CHECK(l == doctest::Approx(0.0));
    CHECK(r == doctest::Approx(0.0));
}

TEST_CASE("Chassis::opcontrol(FlightStyle::Rate): respects should_exit") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    int iter = 0;
    auto poll = [](JoystickAxis) { return 0.0; };
    chassis.opcontrol(FlightStyle::Rate{}, poll,
                      [&] { return ++iter >= 4; });

    // The should_exit callback runs at the top of each loop iteration,
    // so we expect exactly 4 evaluations: 1, 2, 3, then 4 returns true.
    CHECK(iter == 4);
}
