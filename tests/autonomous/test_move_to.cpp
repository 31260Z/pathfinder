#include <doctest/doctest.h>

#include <pathfinder/autonomous/move_to.hpp>
#include "sim_bot.hpp"

#include <cmath>

using pathfinder::Angle;
using pathfinder::ExitConditions;
using pathfinder::MoveToPoint;
using pathfinder::Pid;
using pathfinder::Pose2;
using pathfinder::Vector2;
using pathfinder::distance;
using pathfinder_test::SimBot;

namespace {

MoveToPoint::Options sane_opts() {
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

constexpr double k_dt = 0.01;
constexpr int    k_max_steps = 4000;

} // namespace

TEST_CASE("MoveToPoint: bot starting on the line drives to the target") {
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{36.0, 0.0};
    MoveToPoint ctrl(start, target, sane_opts());

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target) < 1.0);
}

TEST_CASE("MoveToPoint: cross-track error rejection — bot starts off the line") {
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{36.0, 0.0};
    MoveToPoint ctrl(start, target, sane_opts());

    // Bot starts 4 inches above the line (in +Y).
    SimBot bot{Pose2{0.0, 4.0, Angle{0.0}}};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target) < 1.5);
}

TEST_CASE("MoveToPoint: reverse mode drives rear-first to target") {
    Pose2  start{0.0, 0.0, Angle{0.0}};  // facing +X
    Vector2 target{-24.0, 0.0};          // target is behind
    auto    opts = sane_opts();
    opts.reverse = true;
    MoveToPoint ctrl(start, target, opts);

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target) < 1.5);
    // Bot should still be facing roughly +X (reverse drive doesn't rotate it).
    const double h = std::abs(bot.pose.heading.normalize_signed().rad);
    CHECK(h < 0.6);  // within ~35deg of original heading
}

TEST_CASE("MoveToPoint: done() persists once exit triggers, output is zero") {
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{12.0, 0.0};
    MoveToPoint ctrl(start, target, sane_opts());

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    REQUIRE(ctrl.done());
    auto cmd = ctrl.update(bot.pose, k_dt);
    CHECK(cmd.done);
    CHECK(cmd.forward_velocity_ips == doctest::Approx(0.0));
    CHECK(cmd.angular_velocity_dps == doctest::Approx(0.0));
}

TEST_CASE("MoveToPoint: heading converges toward line direction (no cross-track perturbation)") {
    Pose2  start{0.0, 0.0, Angle::degrees(-45.0)};   // facing wrong way
    Vector2 target{20.0, 0.0};
    MoveToPoint ctrl(start, target, sane_opts());

    SimBot bot{start};
    for (int i = 0; i < k_max_steps && !ctrl.done(); ++i) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target) < 1.5);
}
