#include <doctest/doctest.h>

#include <pathfinder/autonomous/turn_to.hpp>
#include "sim_bot.hpp"

#include <cmath>

using pathfinder::Angle;
using pathfinder::ExitConditions;
using pathfinder::Pid;
using pathfinder::Pose2;
using pathfinder::TurnTo;
using pathfinder::TurnToPoint;
using pathfinder::Vector2;
using pathfinder::shortest_angle;
using pathfinder::k_pi;
using pathfinder_test::SimBot;

namespace {

TurnTo::Options sane_opts() {
    TurnTo::Options o{};
    o.heading = Pid::Gains{6.0, 0.0, 0.3};
    o.exit = ExitConditions::Spec{
        .small_error            = 0.5,   // degrees
        .small_error_timeout_ms = 100.0,
        .large_error            = 2.0,
        .large_error_timeout_ms = 600.0,
        .absolute_timeout_ms    = 10000.0,
    };
    o.max_angular_dps = 360.0;
    return o;
}

constexpr double k_dt = 0.01;
constexpr int    k_max_steps = 3000;

} // namespace

TEST_CASE("TurnTo Auto: shortest path across wraparound (-170deg -> +170deg)") {
    Pose2 start{0.0, 0.0, Angle::degrees(-170.0)};
    TurnTo ctrl(Angle::degrees(170.0), sane_opts());

    SimBot bot{start};
    int steps = 0;
    double total_rot = 0.0;
    pathfinder::Angle prev = bot.pose.heading;
    while (!ctrl.done() && steps++ < k_max_steps) {
        auto cmd = ctrl.update(bot.pose, k_dt);
        bot.step(cmd, k_dt);
        total_rot += shortest_angle(prev, bot.pose.heading).rad;
        prev = bot.pose.heading;
    }
    CHECK(ctrl.done());
    // Should rotate ~20 degrees, NOT 340.
    CHECK(std::abs(total_rot) < (60.0 * pathfinder::k_deg_to_rad));
}

TEST_CASE("TurnTo CW: forces long-way-around when shortest is CCW") {
    Pose2 start{0.0, 0.0, Angle::degrees(-30.0)};
    // Shortest from -30 to +30 is CCW (+60); CW direction must go -300.
    TurnTo ctrl(Angle::degrees(30.0), sane_opts(), TurnTo::Direction::CW);

    SimBot bot{start};
    int steps = 0;
    double total_rot = 0.0;
    pathfinder::Angle prev = bot.pose.heading;
    while (!ctrl.done() && steps++ < k_max_steps) {
        auto cmd = ctrl.update(bot.pose, k_dt);
        bot.step(cmd, k_dt);
        total_rot += shortest_angle(prev, bot.pose.heading).rad;
        prev = bot.pose.heading;
    }
    CHECK(ctrl.done());
    // CW = negative omega = negative-heading-direction; total rotation should be ~-300 deg.
    CHECK(total_rot < 0.0);
    CHECK(std::abs(total_rot) > (180.0 * pathfinder::k_deg_to_rad));
}

TEST_CASE("TurnTo CCW: forces long way around when shortest is CW") {
    Pose2 start{0.0, 0.0, Angle::degrees(30.0)};
    TurnTo ctrl(Angle::degrees(-30.0), sane_opts(), TurnTo::Direction::CCW);

    SimBot bot{start};
    int steps = 0;
    double total_rot = 0.0;
    pathfinder::Angle prev = bot.pose.heading;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
        total_rot += shortest_angle(prev, bot.pose.heading).rad;
        prev = bot.pose.heading;
    }
    CHECK(ctrl.done());
    CHECK(total_rot > 0.0);
    CHECK(std::abs(total_rot) > (180.0 * pathfinder::k_deg_to_rad));
}

TEST_CASE("TurnTo done() persists, zero output thereafter") {
    Pose2 start{0.0, 0.0, Angle::degrees(0.0)};
    TurnTo ctrl(Angle::degrees(45.0), sane_opts());

    SimBot bot{start};
    for (int i = 0; i < k_max_steps && !ctrl.done(); ++i) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    REQUIRE(ctrl.done());
    auto cmd = ctrl.update(bot.pose, k_dt);
    CHECK(cmd.done);
    CHECK(cmd.forward_velocity_ips == doctest::Approx(0.0));
    CHECK(cmd.angular_velocity_dps == doctest::Approx(0.0));
}

TEST_CASE("TurnToPoint: bot turns to face the given point") {
    Pose2 start{0.0, 0.0, Angle{0.0}};
    Vector2 target{0.0, 10.0};   // pi/2 from current heading
    TurnToPoint ctrl(target, sane_opts());

    SimBot bot{start};
    for (int i = 0; i < k_max_steps && !ctrl.done(); ++i) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    const double err = std::abs(shortest_angle(bot.pose.heading, Angle{k_pi / 2.0}).rad);
    CHECK(err < (1.0 * pathfinder::k_deg_to_rad));
}
