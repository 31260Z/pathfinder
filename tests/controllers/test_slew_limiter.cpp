#include <doctest/doctest.h>
#include <pathfinder/controllers/slew_limiter.hpp>

#include <cmath>

using pathfinder::SlewLimiter;

TEST_CASE("First update snaps to target") {
    SlewLimiter slew(1.0);
    CHECK(slew.update(100.0, 0.01) == doctest::Approx(100.0));
    CHECK(slew.current() == doctest::Approx(100.0));
}

TEST_CASE("Output rate never exceeds limit") {
    SlewLimiter slew(2.0);
    slew.reset(0.0);

    double prev = 0.0;
    constexpr double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        const double out = slew.update(1000.0, dt);
        const double rate = std::abs(out - prev) / dt;
        CHECK(rate <= doctest::Approx(2.0).epsilon(1e-9));
        prev = out;
    }
}

TEST_CASE("Ramping up takes expected duration") {
    SlewLimiter slew(10.0);
    slew.reset(0.0);

    constexpr double dt = 0.01;
    constexpr int steps = 100;
    for (int i = 0; i < steps; ++i) {
        slew.update(50.0, dt);
    }
    CHECK(slew.current() == doctest::Approx(10.0));
}

TEST_CASE("Ramping down works symmetrically") {
    SlewLimiter slew(5.0);
    slew.reset(20.0);

    constexpr double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        slew.update(0.0, dt);
    }
    CHECK(slew.current() == doctest::Approx(10.0));
}

TEST_CASE("Target reachable when within rate budget") {
    SlewLimiter slew(100.0);
    slew.reset(0.0);
    const double out = slew.update(0.5, 0.01);
    CHECK(out == doctest::Approx(0.5));
}

TEST_CASE("Reaches target exactly without overshoot") {
    SlewLimiter slew(10.0);
    slew.reset(0.0);

    constexpr double dt = 0.1;
    constexpr double target = 5.0;
    for (int i = 0; i < 100; ++i) {
        slew.update(target, dt);
    }
    CHECK(slew.current() == doctest::Approx(target));
}

TEST_CASE("reset(value) sets current and disables first-snap") {
    SlewLimiter slew(1.0);
    slew.reset(7.5);
    CHECK(slew.current() == doctest::Approx(7.5));
    const double out = slew.update(1000.0, 1.0);
    CHECK(out == doctest::Approx(8.5));
}

TEST_CASE("reset() default zeros and disables first-snap") {
    SlewLimiter slew(2.0);
    slew.reset();
    CHECK(slew.current() == doctest::Approx(0.0));
    const double out = slew.update(100.0, 1.0);
    CHECK(out == doctest::Approx(2.0));
}

TEST_CASE("dt <= 0 holds current") {
    SlewLimiter slew(1.0);
    slew.reset(3.0);
    CHECK(slew.update(100.0, 0.0) == doctest::Approx(3.0));
    CHECK(slew.update(100.0, -1.0) == doctest::Approx(3.0));
}

TEST_CASE("Reverses direction without overshoot") {
    SlewLimiter slew(4.0);
    slew.reset(0.0);

    constexpr double dt = 0.5;
    slew.update(2.0, dt);
    CHECK(slew.current() == doctest::Approx(2.0));
    const double out = slew.update(-2.0, dt);
    CHECK(out == doctest::Approx(0.0));
}
