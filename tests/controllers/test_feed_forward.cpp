#include <doctest/doctest.h>
#include <pathfinder/controllers/feed_forward.hpp>

using pathfinder::FeedForward;

TEST_CASE("kV * v_target dominates when only kV is set") {
    FeedForward ff{.kV = 0.15};
    CHECK(ff.compute(60.0, 0.0) == doctest::Approx(9.0));
    CHECK(ff.compute(-60.0, 0.0) == doctest::Approx(-9.0));
}

TEST_CASE("kA * a_target adds linear acceleration term") {
    FeedForward ff{.kV = 0.0, .kA = 0.02};
    CHECK(ff.compute(0.0, 100.0) == doctest::Approx(2.0));
    CHECK(ff.compute(0.0, -50.0) == doctest::Approx(-1.0));
}

TEST_CASE("kV and kA combine linearly") {
    FeedForward ff{.kV = 0.1, .kA = 0.05};
    CHECK(ff.compute(20.0, 10.0) == doctest::Approx(0.1 * 20.0 + 0.05 * 10.0));
}

TEST_CASE("kS adds in direction of v_target") {
    FeedForward ff{.kV = 0.0, .kA = 0.0, .kS = 0.5};
    CHECK(ff.compute( 1.0, 0.0) == doctest::Approx( 0.5));
    CHECK(ff.compute(-1.0, 0.0) == doctest::Approx(-0.5));
}

TEST_CASE("Zero v_target produces zero static-friction term") {
    FeedForward ff{.kS = 1.5};
    CHECK(ff.compute(0.0, 100.0) == doctest::Approx(0.0));
}

TEST_CASE("kS adds to kV * v_target with same sign") {
    FeedForward ff{.kV = 0.1, .kS = 0.3};
    CHECK(ff.compute( 10.0, 0.0) == doctest::Approx( 1.3));
    CHECK(ff.compute(-10.0, 0.0) == doctest::Approx(-1.3));
}

TEST_CASE("Defaults to zero output for zero gains") {
    FeedForward ff{};
    CHECK(ff.compute(50.0, 25.0) == doctest::Approx(0.0));
    CHECK(ff.compute(0.0, 0.0) == doctest::Approx(0.0));
}

TEST_CASE("All three terms combine") {
    FeedForward ff{.kV = 0.2, .kA = 0.05, .kS = 0.1};
    const double v = 10.0, a = 4.0;
    CHECK(ff.compute(v, a) == doctest::Approx(0.1 + 0.2 * v + 0.05 * a));
}
