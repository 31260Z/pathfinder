#include <doctest/doctest.h>

#include <pathfinder/autonomous/boomerang.hpp>
#include "sim_bot.hpp"

#include <cmath>

using pathfinder::Angle;
using pathfinder::Boomerang;
using pathfinder::ExitConditions;
using pathfinder::Pid;
using pathfinder::Pose2;
using pathfinder::Vector2;
using pathfinder::distance;
using pathfinder::shortest_angle;
using pathfinder_test::SimBot;

namespace {

Boomerang::Options sane_opts(double lead = 0.3) {
    Boomerang::Options o{};
    o.along_track = Pid::Gains{4.0, 0.0, 0.2};
    o.cross_track = Pid::Gains{0.5, 0.0, 0.02};
    o.heading     = Pid::Gains{6.0, 0.0, 0.4};
    o.exit = ExitConditions::Spec{
        .small_error            = 0.7,
        .small_error_timeout_ms = 100.0,
        .large_error            = 2.0,
        .large_error_timeout_ms = 800.0,
        .absolute_timeout_ms    = 30000.0,
    };
    o.max_forward_ips = 50.0;
    o.max_angular_dps = 360.0;
    o.lead            = lead;
    return o;
}

constexpr double k_dt = 0.01;
constexpr int    k_max_steps = 6000;

double max_path_length(SimBot& bot, Boomerang& ctrl) {
    double path = 0.0;
    Vector2 prev = bot.pose.translation();
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
        path += distance(prev, bot.pose.translation());
        prev = bot.pose.translation();
    }
    return path;
}

} // namespace

TEST_CASE("Boomerang: bot ends near target pose with target heading") {
    Pose2 start{0.0, 0.0, Angle{0.0}};
    Pose2 target{36.0, 12.0, Angle::degrees(0.0)};
    Boomerang ctrl(start, target, sane_opts());

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target.translation()) < 1.5);
    CHECK(std::abs(shortest_angle(bot.pose.heading, target.heading).rad)
          < (15.0 * pathfinder::k_deg_to_rad));
}

TEST_CASE("Boomerang: smaller lead -> shorter path than larger lead (more direct)") {
    Pose2 start{0.0, 0.0, Angle{0.0}};
    Pose2 target{36.0, 12.0, Angle::degrees(45.0)};

    SimBot bot_a{start};
    Boomerang ctrl_a(start, target, sane_opts(0.05));
    const double path_small = max_path_length(bot_a, ctrl_a);

    SimBot bot_b{start};
    Boomerang ctrl_b(start, target, sane_opts(0.8));
    const double path_large = max_path_length(bot_b, ctrl_b);

    CHECK(ctrl_a.done());
    CHECK(ctrl_b.done());
    // Larger lead places the carrot farther from the target in the early
    // phase, producing more arc and a longer travelled path.
    CHECK(path_small <= path_large + 0.5);
}

TEST_CASE("Boomerang: reverse mode lands at pose driving rear-first") {
    Pose2 start{0.0, 0.0, Angle{0.0}};      // facing +X
    Pose2 target{-36.0, 0.0, Angle{0.0}};    // target is behind, same heading
    auto opts = sane_opts(0.3);
    opts.reverse = true;
    Boomerang ctrl(start, target, opts);

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), target.translation()) < 2.0);
    // Heading should still be roughly facing +X (we drove backward).
    CHECK(std::abs(shortest_angle(bot.pose.heading, target.heading).rad)
          < (30.0 * pathfinder::k_deg_to_rad));
}
