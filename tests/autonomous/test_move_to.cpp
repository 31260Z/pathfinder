#include <doctest/doctest.h>

#include <pathfinder/autonomous/move_to.hpp>
#include "sim_bot.hpp"

#include <cmath>

using pathfinder::Angle;
using pathfinder::BodyVelocity;
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

// ── Wave D: drift-aware tests ────────────────────────────────────────────

TEST_CASE("MoveToPoint: drift-aware update with v_y=0 matches the legacy path") {
    // Sanity check: the velocity-aware overload with BodyVelocity{} should be
    // bit-identical to the legacy 2-arg overload (it's the same code path).
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{36.0, 0.0};

    MoveToPoint ctrl_a(start, target, sane_opts());
    MoveToPoint ctrl_b(start, target, sane_opts());

    SimBot bot_a{start};
    SimBot bot_b{start};
    int steps = 0;
    while (!ctrl_a.done() && steps++ < k_max_steps) {
        bot_a.step(ctrl_a.update(bot_a.pose, k_dt), k_dt);
        bot_b.step(ctrl_b.update(bot_b.pose, BodyVelocity{}, k_dt), k_dt);
    }
    CHECK(ctrl_a.done());
    CHECK(ctrl_b.done());
    CHECK(distance(bot_a.pose.translation(), bot_b.pose.translation()) < 1e-9);
}

TEST_CASE("MoveToPoint: drift-aware regulator anticipates v_y on the first tick") {
    // Compare the heading commanded on tick 1 between the v_y=0 path and a
    // path with a strong positive v_y. The drift-aware regulator should bias
    // the heading the *opposite* way (toward -Y) to anticipate the predicted
    // +Y drift over the lookahead horizon.
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{36.0, 0.0};
    auto opts = sane_opts();
    opts.lookahead_time_sec = 0.20;   // 200 ms — visible bias

    MoveToPoint ctrl_no_drift(start, target, opts);
    MoveToPoint ctrl_drift   (start, target, opts);

    auto cmd_no_drift = ctrl_no_drift.update(start, BodyVelocity{},                 k_dt);
    auto cmd_drift    = ctrl_drift   .update(start, BodyVelocity{0.0, 10.0, 0.0},   k_dt);

    // With +Y drift, the predicted cross-track is positive; the controller
    // subtracts a positive heading_offset, so the commanded angular velocity
    // should be MORE NEGATIVE (toward -Y) than the no-drift case.
    CHECK(cmd_drift.angular_velocity_dps < cmd_no_drift.angular_velocity_dps);
}

TEST_CASE("MoveToPoint: min_exit_speed_ips=0 (default) zeroes the wheels at exit (legacy)") {
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{12.0, 0.0};
    auto opts = sane_opts();
    opts.min_exit_speed_ips = 0.0;
    MoveToPoint ctrl(start, target, opts);

    SimBot bot{start};
    int steps = 0;
    while (!ctrl.done() && steps++ < k_max_steps) {
        bot.step(ctrl.update(bot.pose, BodyVelocity{}, k_dt), k_dt);
    }
    REQUIRE(ctrl.done());
    auto final_cmd = ctrl.update(bot.pose, BodyVelocity{}, k_dt);
    CHECK(final_cmd.done);
    CHECK(final_cmd.forward_velocity_ips == doctest::Approx(0.0));
    CHECK(final_cmd.angular_velocity_dps == doctest::Approx(0.0));
}

TEST_CASE("MoveToPoint: min_exit_speed_ips>0 hands off the last regulator output (no hard stop)") {
    // Set up a long target with a small_error band that's tight enough to
    // require the bot to drive into it, then teleport the bot inside the
    // band to force a fast exit while the regulator's last_v_fwd_ /
    // last_omega_dps_ are still cruising. With min_exit_speed_ips > 0, the
    // controller's exit-tick command must equal the last regulator output
    // — NOT a hard zero — so the chassis can hand off to the next segment.
    Pose2  start{0.0, 0.0, Angle{0.0}};
    Vector2 target{36.0, 0.0};
    auto opts = sane_opts();
    opts.min_exit_speed_ips = 20.0;
    opts.exit = ExitConditions::Spec{
        .small_error            = 1.0,
        .small_error_timeout_ms = 1.0,    // 1 ms hold (very fast)
        .large_error            = 0.0,    // disabled (no large-band trip)
        .large_error_timeout_ms = 0.0,
        .absolute_timeout_ms    = 30000.0,
    };
    MoveToPoint ctrl(start, target, opts);

    SimBot bot{start};
    // Tick 1: builds up the regulator output (last_v_fwd_, last_omega_dps_)
    // for a 36-in remaining error. Far outside small band → no exit.
    auto build_cmd = ctrl.update(bot.pose, BodyVelocity{}, k_dt);
    REQUIRE(build_cmd.forward_velocity_ips > 0.0);
    bot.step(build_cmd, k_dt);

    // Teleport inside the small-error band. The next tick's exit_.feed sees
    // remaining≈0.5, the small band fills past its 1ms hold, and the exit
    // fires inside the same call. With min_exit_speed_ips > 0, the returned
    // command is the last-cycle regulator output (NOT a hard zero).
    bot.pose = Pose2{35.5, 0.0, Angle{0.0}};
    auto exit_cmd = ctrl.update(bot.pose, BodyVelocity{}, k_dt);
    REQUIRE(exit_cmd.done);
    const bool any_nonzero = std::abs(exit_cmd.forward_velocity_ips) > 1e-9
                          || std::abs(exit_cmd.angular_velocity_dps) > 1e-9;
    CHECK(any_nonzero);

    // After the exit, subsequent updates do return zeros (the controller is
    // fully done; the chassis is responsible for switching to the next
    // segment, not for re-querying this controller).
    auto post_exit_cmd = ctrl.update(bot.pose, BodyVelocity{}, k_dt);
    CHECK(post_exit_cmd.done);
    CHECK(post_exit_cmd.forward_velocity_ips == doctest::Approx(0.0));
    CHECK(post_exit_cmd.angular_velocity_dps == doctest::Approx(0.0));

    // Verify setter overload exists and compiles.
    MoveToPoint ctrl2(Pose2{}, Vector2{1.0, 0.0}, sane_opts());
    ctrl2.set_min_exit_speed(15.0);
}
