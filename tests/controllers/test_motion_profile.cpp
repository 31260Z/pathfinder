#include <doctest/doctest.h>
#include <pathfinder/controllers/motion_profile.hpp>

#include <cmath>

using pathfinder::motion_profile::SCurve;
using pathfinder::motion_profile::Sample;
using pathfinder::motion_profile::Trapezoidal;

TEST_CASE("Trapezoidal: trapezoid case (long distance)") {
    Trapezoidal p{.v_max = 10.0, .a_max = 5.0, .start_pos = 0.0, .end_pos = 100.0};
    const double T = p.total_duration();
    // t_a = 2s (each), d_accel = 10. Cruise distance = 80, t_c = 8s. Total = 12s.
    CHECK(T == doctest::Approx(12.0));

    CHECK(p.sample(0.0).position == doctest::Approx(0.0));
    CHECK(p.sample(0.0).velocity == doctest::Approx(0.0));

    CHECK(p.sample(T).position == doctest::Approx(100.0));
    CHECK(p.sample(T).velocity == doctest::Approx(0.0));

    // Mid-cruise: velocity at v_max, zero acceleration.
    const Sample mid = p.sample(T / 2.0);
    CHECK(mid.velocity == doctest::Approx(10.0));
    CHECK(mid.acceleration == doctest::Approx(0.0));

    // During accel phase, acceleration is +a_max.
    CHECK(p.sample(1.0).acceleration == doctest::Approx(5.0));
    // During decel phase, acceleration is -a_max.
    CHECK(p.sample(11.0).acceleration == doctest::Approx(-5.0));
}

TEST_CASE("Trapezoidal: triangle case (short distance)") {
    Trapezoidal p{.v_max = 100.0, .a_max = 10.0, .start_pos = 0.0, .end_pos = 10.0};
    // d_accel at v_max would be 500, way more than 5. Triangle case.
    // v_peak = sqrt(10 * 10) = 10. t_p = 1. T = 2.
    const double T = p.total_duration();
    CHECK(T == doctest::Approx(2.0));

    CHECK(p.sample(0.0).position == doctest::Approx(0.0));
    CHECK(p.sample(T).position == doctest::Approx(10.0));
    CHECK(p.sample(T).velocity == doctest::Approx(0.0));

    // Peak velocity at apex.
    const Sample peak = p.sample(1.0);
    CHECK(peak.velocity == doctest::Approx(10.0));
}

TEST_CASE("Trapezoidal: reverse motion (end < start)") {
    Trapezoidal p{.v_max = 10.0, .a_max = 5.0, .start_pos = 100.0, .end_pos = 0.0};
    const double T = p.total_duration();
    CHECK(T == doctest::Approx(12.0));

    CHECK(p.sample(0.0).position == doctest::Approx(100.0));
    CHECK(p.sample(T).position == doctest::Approx(0.0));

    // Velocity should be negative during the move.
    const Sample mid = p.sample(T / 2.0);
    CHECK(mid.velocity == doctest::Approx(-10.0));
    CHECK(mid.acceleration == doctest::Approx(0.0));

    CHECK(p.sample(1.0).acceleration == doctest::Approx(-5.0));
    CHECK(p.sample(11.0).acceleration == doctest::Approx(5.0));
}

TEST_CASE("Trapezoidal: clamps t outside [0, total_duration]") {
    Trapezoidal p{.v_max = 10.0, .a_max = 5.0, .start_pos = 0.0, .end_pos = 100.0};
    CHECK(p.sample(-5.0).position == doctest::Approx(0.0));
    CHECK(p.sample(1000.0).position == doctest::Approx(100.0));
}

TEST_CASE("Trapezoidal: zero distance is no motion") {
    Trapezoidal p{.v_max = 10.0, .a_max = 5.0, .start_pos = 5.0, .end_pos = 5.0};
    CHECK(p.total_duration() == doctest::Approx(0.0));
    CHECK(p.sample(0.0).position == doctest::Approx(5.0));
    CHECK(p.sample(1.0).position == doctest::Approx(5.0));
}

TEST_CASE("Trapezoidal: position is monotonic") {
    Trapezoidal p{.v_max = 10.0, .a_max = 5.0, .start_pos = 0.0, .end_pos = 100.0};
    const double T = p.total_duration();
    double prev = -1.0;
    for (int i = 0; i <= 200; ++i) {
        const double t = T * i / 200.0;
        const double pos = p.sample(t).position;
        CHECK(pos >= prev - 1e-9);
        prev = pos;
    }
}

TEST_CASE("SCurve: long distance fits all 7 segments") {
    SCurve p{.v_max = 10.0, .a_max = 5.0, .j_max = 5.0,
             .start_pos = 0.0, .end_pos = 100.0};
    const double T = p.total_duration();
    CHECK(T > 0.0);

    CHECK(p.sample(0.0).position == doctest::Approx(0.0));
    CHECK(p.sample(0.0).velocity == doctest::Approx(0.0));
    CHECK(p.sample(T).position == doctest::Approx(100.0).epsilon(1e-6));
    CHECK(p.sample(T).velocity == doctest::Approx(0.0).epsilon(1e-6));

    // Position monotonic; velocity bounded by v_max; |accel| bounded by a_max.
    double prev_pos = -1.0;
    double max_v = 0.0;
    double max_a_abs = 0.0;
    for (int i = 0; i <= 1000; ++i) {
        const double t = T * i / 1000.0;
        const Sample s = p.sample(t);
        CHECK(s.position >= prev_pos - 1e-9);
        max_v     = std::max(max_v, s.velocity);
        max_a_abs = std::max(max_a_abs, std::abs(s.acceleration));
        prev_pos  = s.position;
    }
    CHECK(max_v <= doctest::Approx(10.0).epsilon(1e-6));
    CHECK(max_v == doctest::Approx(10.0).epsilon(0.01));
    CHECK(max_a_abs <= doctest::Approx(5.0).epsilon(1e-6));
}

TEST_CASE("SCurve: jerk bounded by j_max via finite-difference of acceleration") {
    SCurve p{.v_max = 10.0, .a_max = 5.0, .j_max = 5.0,
             .start_pos = 0.0, .end_pos = 100.0};
    const double T = p.total_duration();
    constexpr int N = 4000;
    const double dt = T / N;

    double prev_a = 0.0;
    double max_jerk = 0.0;
    for (int i = 0; i <= N; ++i) {
        const double t = T * i / N;
        const double a = p.sample(t).acceleration;
        if (i > 0) {
            const double j = std::abs((a - prev_a) / dt);
            max_jerk = std::max(max_jerk, j);
        }
        prev_a = a;
    }
    CHECK(max_jerk <= doctest::Approx(5.0).epsilon(0.05));
}

TEST_CASE("SCurve: short distance falls back to trapezoidal") {
    SCurve p{.v_max = 100.0, .a_max = 10.0, .j_max = 1.0,
             .start_pos = 0.0, .end_pos = 1.0};
    const double T = p.total_duration();
    CHECK(T > 0.0);
    CHECK(p.sample(0.0).position == doctest::Approx(0.0));
    CHECK(p.sample(T).position == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(p.sample(T).velocity == doctest::Approx(0.0).epsilon(1e-6));
}

TEST_CASE("SCurve: reverse motion (end < start)") {
    SCurve p{.v_max = 10.0, .a_max = 5.0, .j_max = 5.0,
             .start_pos = 100.0, .end_pos = 0.0};
    const double T = p.total_duration();
    CHECK(T > 0.0);
    CHECK(p.sample(0.0).position == doctest::Approx(100.0));
    CHECK(p.sample(T).position == doctest::Approx(0.0).epsilon(1e-6));

    double min_v = 0.0;
    for (int i = 0; i <= 200; ++i) {
        const double t = T * i / 200.0;
        min_v = std::min(min_v, p.sample(t).velocity);
    }
    CHECK(min_v == doctest::Approx(-10.0).epsilon(0.01));
}

TEST_CASE("SCurve: zero distance is no motion") {
    SCurve p{.v_max = 10.0, .a_max = 5.0, .j_max = 5.0,
             .start_pos = 7.0, .end_pos = 7.0};
    CHECK(p.total_duration() == doctest::Approx(0.0));
    CHECK(p.sample(0.0).position == doctest::Approx(7.0));
}

TEST_CASE("SCurve: clamps t outside [0, total_duration]") {
    SCurve p{.v_max = 10.0, .a_max = 5.0, .j_max = 5.0,
             .start_pos = 0.0, .end_pos = 50.0};
    CHECK(p.sample(-1.0).position == doctest::Approx(0.0));
    CHECK(p.sample(1e6).position == doctest::Approx(50.0).epsilon(1e-6));
}
