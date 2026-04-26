#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

#include <cmath>

using namespace pathfinder;

namespace {
constexpr double k_pi_local = 3.14159265358979323846;
constexpr double k_eps      = 1e-9;
} // namespace

TEST_CASE("DR: default pose is the origin") {
    DeadReckoning dr({});
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));
    CHECK(p.heading.radians() == doctest::Approx(0.0));
}

TEST_CASE("DR: initial pose is preserved before any updates") {
    Pose2 start{5.0, -2.0, Angle::degrees(30.0)};
    DeadReckoning dr({}, start);
    CHECK(dr.pose() == start);
}

TEST_CASE("DR: first update seeds heading without translating") {
    DeadReckoning dr({}, Pose2{1.0, 2.0, Angle::radians(0.0)});
    // First call: even with a nonzero wheel reading we have no Δθ baseline,
    // so the integrator must not move the pose.
    dr.update(10.0, 0.0, Angle::radians(0.5));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(1.0));
    CHECK(p.y == doctest::Approx(2.0));
    CHECK(p.heading.radians() == doctest::Approx(0.5));
}

TEST_CASE("DR: pure forward at heading 0 advances +X") {
    DeadReckoning dr({});
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(5.0, 0.0, Angle::radians(0.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(5.0));
    CHECK(p.y == doctest::Approx(0.0).epsilon(k_eps));
    CHECK(p.heading.radians() == doctest::Approx(0.0));
}

TEST_CASE("DR: pure forward at heading +pi/2 advances +Y") {
    DeadReckoning dr({}, Pose2{0.0, 0.0, Angle::radians(k_pi_local / 2.0)});
    dr.update(0.0, 0.0, Angle::radians(k_pi_local / 2.0));   // seed
    dr.update(7.5, 0.0, Angle::radians(k_pi_local / 2.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(k_eps));
    CHECK(p.y == doctest::Approx(7.5));
}

TEST_CASE("DR: pure forward at heading +pi reverses to -X") {
    DeadReckoning dr({}, Pose2{0.0, 0.0, Angle::radians(k_pi_local)});
    dr.update(0.0, 0.0, Angle::radians(k_pi_local));   // seed
    dr.update(3.0, 0.0, Angle::radians(k_pi_local));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(-3.0));
    CHECK(p.y == doctest::Approx(0.0).epsilon(k_eps));
}

TEST_CASE("DR: pure forward at arbitrary heading projects correctly") {
    const double theta = k_pi_local / 6.0;   // 30 deg
    DeadReckoning dr({}, Pose2{0.0, 0.0, Angle::radians(theta)});
    dr.update(0.0, 0.0, Angle::radians(theta));   // seed
    dr.update(10.0, 0.0, Angle::radians(theta));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(10.0 * std::cos(theta)));
    CHECK(p.y == doctest::Approx(10.0 * std::sin(theta)));
}

TEST_CASE("DR: pure rotation about bot center leaves x/y unchanged") {
    // Parallel wheel offset 3" right of center. Bot pivots about its center
    // by Δθ = +0.4 rad. Per spec App. A, encoder reads (P_y − y_p)·Δθ; with
    // P=(0,0) and y_p=3, that's −3·0.4 = −1.2.
    DeadReckoning::Config cfg{};
    cfg.parallel_wheel_y_offset_in = 3.0;
    DeadReckoning dr(cfg, Pose2{2.0, -1.0, Angle::radians(0.0)});
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(-3.0 * 0.4, 0.0, Angle::radians(0.4));

    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(2.0).epsilon(k_eps));
    CHECK(p.y == doctest::Approx(-1.0).epsilon(k_eps));
    CHECK(p.heading.radians() == doctest::Approx(0.4));
}

TEST_CASE("DR: pure rotation about bot center, perp wheel offset") {
    // Perp wheel offset 1.5" forward of center. Pure spin about center:
    // encoder reads −(x_q − P_x)·Δθ in body-Y projection? Let me work it
    // again: wheel at (x_q, 0) moves by Δθ·perp((x_q,0)) = (0, Δθ·x_q) in
    // body frame; perp wheel reads body-Y = Δθ·x_q. So for Δθ=0.3, x_q=1.5
    // the perp encoder reads 0.45.
    DeadReckoning::Config cfg{};
    cfg.has_perp_wheel        = true;
    cfg.perp_wheel_x_offset_in = 1.5;
    DeadReckoning dr(cfg);
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(0.0, 0.3 * 1.5, Angle::radians(0.3));

    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(k_eps));
    CHECK(p.y == doctest::Approx(0.0).epsilon(k_eps));
    CHECK(p.heading.radians() == doctest::Approx(0.3));
}

TEST_CASE("DR: lateral perp-wheel reading translates in world Y at heading 0") {
    DeadReckoning::Config cfg{};
    cfg.has_perp_wheel = true;
    DeadReckoning dr(cfg);
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(0.0, 4.0, Angle::radians(0.0));   // pure body-Y motion, no rotation
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(k_eps));
    CHECK(p.y == doctest::Approx(4.0));
}

TEST_CASE("DR: lateral perp-wheel reading at heading +pi/2 translates in world -X") {
    // body-Y at heading π/2 maps to world (-1, 0): R(π/2)·(0,1) = (-1, 0).
    DeadReckoning::Config cfg{};
    cfg.has_perp_wheel = true;
    DeadReckoning dr(cfg, Pose2{0.0, 0.0, Angle::radians(k_pi_local / 2.0)});
    dr.update(0.0, 0.0, Angle::radians(k_pi_local / 2.0));   // seed
    dr.update(0.0, 2.0, Angle::radians(k_pi_local / 2.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(-2.0));
    CHECK(p.y == doctest::Approx(0.0).epsilon(k_eps));
}

TEST_CASE("DR: heading wraparound — Δθ uses shortest path") {
    // Heading jumps from +pi - 0.1 to -pi + 0.1: naive subtraction would give
    // ~-2pi+0.2; the correct shortest delta is +0.2.
    DeadReckoning dr({});
    dr.update(0.0, 0.0, Angle::radians(k_pi_local - 0.1));   // seed
    // Pure forward of 1.0 inch with the small wraparound; mid-heading
    // should be near +pi, so motion should be (cos(pi), sin(pi)) ≈ (-1, 0).
    dr.update(1.0, 0.0, Angle::radians(-k_pi_local + 0.1));
    const Pose2 p = dr.pose();
    // Mid-heading = (pi - 0.1) + 0.1 = pi exactly.
    CHECK(p.x == doctest::Approx(std::cos(k_pi_local)));
    CHECK(p.y == doctest::Approx(std::sin(k_pi_local)).epsilon(k_eps));
    // End heading is preserved as-is (caller's absolute reading).
    CHECK(p.heading.radians() == doctest::Approx(-k_pi_local + 0.1));
}

TEST_CASE("DR: combined forward + rotation uses mid-heading") {
    // Bot starts at origin facing 0. Drives 2" forward while heading goes
    // 0 → π/2. Mid-heading = π/4. Body-frame Δ is (2, 0), no perp wheel.
    // World-frame: 2 * (cos π/4, sin π/4) ≈ (1.4142, 1.4142).
    DeadReckoning dr({});
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(2.0, 0.0, Angle::radians(k_pi_local / 2.0));
    const Pose2 p = dr.pose();
    const double expected = 2.0 * std::cos(k_pi_local / 4.0);   // ≈ 1.41421356
    CHECK(p.x == doctest::Approx(expected));
    CHECK(p.y == doctest::Approx(expected));
    CHECK(p.heading.radians() == doctest::Approx(k_pi_local / 2.0));
}

TEST_CASE("DR: many small steps approximate a circular arc") {
    // Drive a 90° arc of radius R=10 in many tiny steps. Closed-form: bot
    // starts at (0,0,0), pivots around (0, 10) world-side; ends at (10, 10)
    // facing π/2.
    DeadReckoning dr({});
    constexpr int n = 1000;
    constexpr double R = 10.0;
    const double total_theta = k_pi_local / 2.0;
    const double arc_len     = R * total_theta;
    const double d_s         = arc_len / n;
    const double d_theta     = total_theta / n;

    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    double heading = 0.0;
    for (int i = 0; i < n; ++i) {
        heading += d_theta;
        dr.update(d_s, 0.0, Angle::radians(heading));
    }
    const Pose2 p = dr.pose();
    // Mid-heading integration is 2nd-order, so 1000 steps is overkill; gets
    // sub-millimeter accuracy.
    CHECK(p.x == doctest::Approx(R).epsilon(1e-4));
    CHECK(p.y == doctest::Approx(R).epsilon(1e-4));
    CHECK(p.heading.radians() == doctest::Approx(k_pi_local / 2.0));
}

TEST_CASE("DR: set_pose teleports without disturbing integration history") {
    DeadReckoning dr({});
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed prev_heading_ = 0
    dr.update(1.0, 0.0, Angle::radians(0.0));   // pose now (1, 0, 0)

    dr.set_pose(Pose2{50.0, 50.0, Angle::radians(0.0)});
    // Next update: prev_heading_ should still be 0 from the previous update,
    // not reset. Driving 2" forward should now land us at (52, 50).
    dr.update(2.0, 0.0, Angle::radians(0.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(52.0));
    CHECK(p.y == doctest::Approx(50.0).epsilon(k_eps));
}

TEST_CASE("DR: reset clears prev-heading without moving the pose") {
    DeadReckoning dr({}, Pose2{3.0, 4.0, Angle::radians(0.5)});
    dr.update(0.0, 0.0, Angle::radians(0.5));   // seed
    dr.update(5.0, 0.0, Angle::radians(0.5));   // moves the pose

    const Pose2 before_reset = dr.pose();
    dr.reset();
    CHECK(dr.pose() == before_reset);   // pose unchanged

    // After reset, the next update is a "first" update again — should NOT
    // translate even with a nonzero wheel reading.
    dr.update(99.0, 0.0, Angle::radians(0.5));
    CHECK(dr.pose() == before_reset);
}

TEST_CASE("DR: backward motion (negative encoder) goes the right way") {
    DeadReckoning dr({});
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(-4.0, 0.0, Angle::radians(0.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(-4.0));
    CHECK(p.y == doctest::Approx(0.0).epsilon(k_eps));
}

TEST_CASE("DR: combined parallel + perp encoder readings") {
    // Heading 0, no rotation. Δs_par = 3, Δs_perp = -2 → body Δ = (3, -2),
    // world Δ = (3, -2) at heading 0.
    DeadReckoning::Config cfg{};
    cfg.has_perp_wheel = true;
    DeadReckoning dr(cfg);
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(3.0, -2.0, Angle::radians(0.0));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(3.0));
    CHECK(p.y == doctest::Approx(-2.0));
}

TEST_CASE("DR: spec App. A wheel-finder scenario — bot pivots about a corner") {
    // The wheel-finder tracks the BOT CENTER. All offsets passed to DR's
    // config are measured *relative to the center of rotation*. So we work in
    // a center-origin body frame here, regardless of what corner-origin the
    // user has chosen for their `Bot` definition.
    //
    // Setup: parallel wheel at center-relative (x_p, y_p) = (−5, −3) (i.e.
    // 5" behind and 3" left of center). Perp wheel at (x_q, y_q) = (−7, 0).
    // User pins the back-left corner; for an 18×18 bot whose center we're
    // tracking, that corner sits at center-relative body-frame (−9, −9).
    //
    // Per spec App. A:
    //   parallel encoder = (P_y − y_p) · Δθ = (−9 − (−3)) · 0.5 = −3.0.
    //   perp encoder     = (x_q − P_x) · Δθ = (−7 − (−9)) · 0.5 = +1.0.
    DeadReckoning::Config cfg{};
    cfg.parallel_wheel_y_offset_in = -3.0;
    cfg.has_perp_wheel              = true;
    cfg.perp_wheel_x_offset_in      = -7.0;

    // Bot center starts at world origin facing +X. The back-left corner
    // (= world pivot) is therefore at world (−9, −9) — no rotation yet, so
    // body and world axes line up.
    DeadReckoning dr(cfg);
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed

    const double d_theta  = 0.5;
    const double par_enc  = (-9.0 - (-3.0)) * d_theta;   // −3.0
    const double perp_enc = (-7.0 - (-9.0)) * d_theta;   // +1.0
    dr.update(par_enc, perp_enc, Angle::radians(d_theta));

    // Closed-form: bot center pivots about world point (−9, −9):
    //   center' = pivot + R(Δθ) · (center − pivot)
    //           = (−9, −9) + R(0.5) · (9, 9).
    const double cos05    = std::cos(d_theta);
    const double sin05    = std::sin(d_theta);
    const double expect_x = -9.0 + 9.0 * cos05 - 9.0 * sin05;
    const double expect_y = -9.0 + 9.0 * sin05 + 9.0 * cos05;

    const Pose2 p = dr.pose();
    // Single-step mid-heading integration on a non-infinitesimal Δθ truncates
    // at O(Δθ³); loosen the tolerance for the one-shot case. The refined
    // sub-test below verifies asymptotic correctness.
    CHECK(p.x == doctest::Approx(expect_x).epsilon(0.05));
    CHECK(p.y == doctest::Approx(expect_y).epsilon(0.05));
    CHECK(p.heading.radians() == doctest::Approx(d_theta));
}

TEST_CASE("DR: spec App. A scenario, refined into many small steps") {
    // Same scenario as above, discretized so the mid-heading integrator hits
    // the closed-form answer to high precision.
    DeadReckoning::Config cfg{};
    cfg.parallel_wheel_y_offset_in = -3.0;
    cfg.has_perp_wheel              = true;
    cfg.perp_wheel_x_offset_in      = -7.0;

    DeadReckoning dr(cfg);
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed

    constexpr int    n           = 500;
    constexpr double total_theta = 0.5;
    const double     d_theta     = total_theta / n;
    const double     par_step    = (-9.0 - (-3.0)) * d_theta;   // each tick reads (P_y − y_p)·Δθ
    const double     perp_step   = (-7.0 - (-9.0)) * d_theta;   // and (x_q − P_x)·Δθ

    double heading = 0.0;
    for (int i = 0; i < n; ++i) {
        heading += d_theta;
        dr.update(par_step, perp_step, Angle::radians(heading));
    }

    const double cos05    = std::cos(total_theta);
    const double sin05    = std::sin(total_theta);
    const double expect_x = -9.0 + 9.0 * cos05 - 9.0 * sin05;
    const double expect_y = -9.0 + 9.0 * sin05 + 9.0 * cos05;

    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(expect_x).epsilon(1e-4));
    CHECK(p.y == doctest::Approx(expect_y).epsilon(1e-4));
    CHECK(p.heading.radians() == doctest::Approx(total_theta));
}

TEST_CASE("DR: zero-Δθ step is a clean straight-line projection") {
    // Make sure the d_theta=0 path doesn't do anything weird (no NaNs from
    // 0/0, no off-by-one in the mid-heading expression).
    DeadReckoning dr({}, Pose2{0.0, 0.0, Angle::radians(0.25)});
    dr.update(0.0, 0.0, Angle::radians(0.25));   // seed
    dr.update(2.0, 0.0, Angle::radians(0.25));
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(2.0 * std::cos(0.25)));
    CHECK(p.y == doctest::Approx(2.0 * std::sin(0.25)));
    CHECK(std::isfinite(p.x));
    CHECK(std::isfinite(p.y));
}

TEST_CASE("DR: perp wheel disabled means lateral encoder is ignored") {
    // has_perp_wheel=false → d_y = 0 regardless of perp_wheel_delta_in.
    DeadReckoning dr({});   // has_perp_wheel defaults to false
    dr.update(0.0, 0.0, Angle::radians(0.0));   // seed
    dr.update(0.0, 99.0, Angle::radians(0.0));  // bogus perp reading
    const Pose2 p = dr.pose();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));
}
