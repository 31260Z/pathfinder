#include <doctest/doctest.h>

#include <pathfinder/autonomous/pure_pursuit.hpp>
#include "sim_bot.hpp"

#include <cmath>
#include <vector>

using pathfinder::Angle;
using pathfinder::ExitConditions;
using pathfinder::Pid;
using pathfinder::Pose2;
using pathfinder::PurePursuit;
using pathfinder::Vector2;
using pathfinder::Waypoint;
using pathfinder::distance;
using pathfinder::catmull_rom::Path;
using pathfinder_test::SimBot;

namespace {

PurePursuit::Options sane_opts() {
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

constexpr double k_dt = 0.01;
constexpr int    k_max_steps = 8000;

} // namespace

TEST_CASE("PurePursuit: bot follows simple multi-waypoint path and exits") {
    std::vector<Vector2> wpts{
        {0, 0}, {12, 4}, {24, -4}, {36, 0}, {48, 8},
    };
    Path path(wpts);
    PurePursuit ctrl(std::move(path), sane_opts());

    SimBot bot{Pose2{0.0, 0.0, Angle{0.0}}};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), wpts.back()) < 1.5);
}

TEST_CASE("PurePursuit: lookahead clamps to endpoint near end of path") {
    std::vector<Vector2> wpts{
        {0, 0}, {10, 0}, {20, 0},
    };
    Path path(std::move(wpts));
    PurePursuit ctrl(std::move(path), sane_opts());

    // Place bot near the end — lookahead would project past the endpoint.
    SimBot bot{Pose2{18.0, 0.5, Angle{0.0}}};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    CHECK(distance(bot.pose.translation(), Vector2{20.0, 0.0}) < 1.5);
}

TEST_CASE("PurePursuit: on_arrive callbacks fire in order, once each") {
    std::vector<Vector2> wpts{
        {0, 0}, {12, 0}, {24, 0}, {36, 0},
    };
    Path path(wpts);

    std::vector<int> fired_order;
    auto make_cb = [&fired_order](int id) {
        return [&fired_order, id]() { fired_order.push_back(id); };
    };

    std::vector<Waypoint> meta{
        Waypoint{ 0,  0, std::nullopt, std::nullopt, make_cb(0)},
        Waypoint{12,  0, std::nullopt, std::nullopt, make_cb(1)},
        Waypoint{24,  0, std::nullopt, std::nullopt, make_cb(2)},
        Waypoint{36,  0, std::nullopt, std::nullopt, make_cb(3)},
    };
    PurePursuit ctrl(std::move(path), sane_opts(), std::move(meta));

    SimBot bot{Pose2{0.0, 0.0, Angle{0.0}}};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
    }
    CHECK(ctrl.done());
    REQUIRE(fired_order.size() == 4u);
    CHECK(fired_order[0] == 0);
    CHECK(fired_order[1] == 1);
    CHECK(fired_order[2] == 2);
    CHECK(fired_order[3] == 3);
}

TEST_CASE("PurePursuit: per-waypoint speed cap reduces forward speed in that segment") {
    std::vector<Vector2> wpts{
        {0, 0}, {30, 0}, {60, 0},
    };
    Path path(wpts);

    std::vector<Waypoint> meta{
        Waypoint{ 0, 0},
        Waypoint{30, 0, /*speed_cap_ips*/ 5.0},  // throttle approach to mid waypoint
        Waypoint{60, 0},
    };
    auto opts = sane_opts();
    opts.max_forward_ips = 50.0;
    PurePursuit ctrl(std::move(path), opts, std::move(meta));

    SimBot bot{Pose2{0.0, 0.0, Angle{0.0}}};
    double max_v = 0.0;
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        auto cmd = ctrl.update(bot.pose, k_dt);
        // Only sample velocity while we're in the early segment (before waypoint 1).
        if (bot.pose.x < 25.0) {
            max_v = std::max(max_v, std::abs(cmd.forward_velocity_ips));
        }
        bot.step(cmd, k_dt);
    }
    CHECK(ctrl.done());
    CHECK(max_v <= 5.0 + 0.01);
}

TEST_CASE("PurePursuit: progress_distance_in is monotonic non-decreasing") {
    std::vector<Vector2> wpts{
        {0, 0}, {20, 5}, {40, -5}, {60, 0},
    };
    Path path(wpts);
    PurePursuit ctrl(std::move(path), sane_opts());

    SimBot bot{Pose2{0.0, 0.0, Angle{0.0}}};
    double last_progress = 0.0;
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, k_dt), k_dt);
        const double now = ctrl.progress_distance_in();
        CHECK(now >= last_progress - 1e-9);
        last_progress = now;
    }
    CHECK(ctrl.done());
}
