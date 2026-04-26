#include <doctest/doctest.h>
#include <pathfinder/controllers/pid.hpp>

#include <cmath>

using pathfinder::Pid;

TEST_CASE("P-only proportional response") {
    Pid pid({.kP = 2.0});
    const double out = pid.update(10.0, 0.0, 0.01);
    CHECK(out == doctest::Approx(20.0));
}

TEST_CASE("P-only sign tracks error sign") {
    Pid pid({.kP = 1.5});
    CHECK(pid.update(0.0, 5.0, 0.01) == doctest::Approx(-7.5));
}

TEST_CASE("First update skips D-term") {
    Pid pid({.kP = 0.0, .kI = 0.0, .kD = 100.0});
    const double out = pid.update(10.0, 0.0, 0.01);
    CHECK(out == doctest::Approx(0.0));
}

TEST_CASE("D-term acts on subsequent samples") {
    Pid pid({.kP = 0.0, .kI = 0.0, .kD = 1.0});
    pid.update(10.0, 0.0, 0.1);
    const double out = pid.update(10.0, 5.0, 0.1);
    CHECK(out == doctest::Approx(-50.0));
}

TEST_CASE("I-term accumulates over time") {
    Pid pid({.kP = 0.0, .kI = 1.0});
    pid.update(1.0, 0.0, 1.0);
    const double out = pid.update(1.0, 0.0, 1.0);
    CHECK(out == doctest::Approx(2.0));
}

TEST_CASE("Output clamping respects symmetric output_max") {
    Pid pid({.kP = 1000.0}, {.output_max = 12.0});
    CHECK(pid.update( 100.0, 0.0, 0.01) == doctest::Approx( 12.0));
    pid.reset();
    CHECK(pid.update(-100.0, 0.0, 0.01) == doctest::Approx(-12.0));
}

TEST_CASE("Integral anti-windup clamps the integrator") {
    Pid pid({.kP = 0.0, .kI = 1.0}, {.integral_clamp = 5.0});
    for (int i = 0; i < 100; ++i) {
        pid.update(10.0, 0.0, 1.0);
    }
    CHECK(pid.update(10.0, 0.0, 1.0) == doctest::Approx(5.0));
}

TEST_CASE("Integral anti-windup symmetric on negative side") {
    Pid pid({.kP = 0.0, .kI = 1.0}, {.integral_clamp = 3.0});
    for (int i = 0; i < 100; ++i) {
        pid.update(-10.0, 0.0, 1.0);
    }
    CHECK(pid.update(-10.0, 0.0, 1.0) == doctest::Approx(-3.0));
}

TEST_CASE("Deadband zeros small errors") {
    Pid pid({.kP = 10.0}, {.deadband = 1.0});
    CHECK(pid.update(0.5,  0.0, 0.01) == doctest::Approx(0.0));
    CHECK(pid.update(-0.9, 0.0, 0.01) == doctest::Approx(0.0));
    CHECK(pid.update(2.0,  0.0, 0.01) == doctest::Approx(20.0));
}

TEST_CASE("reset clears integral and previous-error state") {
    Pid pid({.kP = 0.0, .kI = 1.0, .kD = 1.0});
    for (int i = 0; i < 50; ++i) {
        pid.update(10.0, 0.0, 0.1);
    }
    pid.reset();
    // First update after reset: D-term skipped, integral starts at 0.
    const double out = pid.update(10.0, 0.0, 0.1);
    CHECK(out == doctest::Approx(1.0));
}

TEST_CASE("dt <= 0 returns last output, no divide-by-zero") {
    Pid pid({.kP = 1.0});
    const double a = pid.update(5.0, 0.0, 0.01);
    const double b = pid.update(5.0, 0.0, 0.0);
    CHECK(b == doctest::Approx(a));
    const double c = pid.update(5.0, 0.0, -0.5);
    CHECK(c == doctest::Approx(a));
    CHECK(std::isfinite(b));
    CHECK(std::isfinite(c));
}

TEST_CASE("dt <= 0 on first call returns zero") {
    Pid pid({.kP = 1.0});
    CHECK(pid.update(5.0, 0.0, 0.0) == doctest::Approx(0.0));
}

TEST_CASE("PID step response converges to setpoint") {
    Pid pid({.kP = 0.5, .kI = 0.1, .kD = 0.05});
    double measurement = 0.0;
    constexpr double setpoint = 10.0;
    constexpr double dt = 0.01;

    for (int i = 0; i < 5000; ++i) {
        const double u = pid.update(setpoint, measurement, dt);
        measurement += u * dt;
    }
    CHECK(measurement == doctest::Approx(setpoint).epsilon(0.05));
}

TEST_CASE("set_gains updates gains in place") {
    Pid pid({.kP = 1.0});
    CHECK(pid.gains().kP == doctest::Approx(1.0));
    pid.set_gains({.kP = 2.5, .kI = 0.5});
    CHECK(pid.gains().kP == doctest::Approx(2.5));
    CHECK(pid.gains().kI == doctest::Approx(0.5));
}
