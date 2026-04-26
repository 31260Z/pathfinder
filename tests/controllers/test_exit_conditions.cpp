#include <doctest/doctest.h>
#include <pathfinder/controllers/exit_conditions.hpp>

using pathfinder::ExitConditions;

TEST_CASE("Small-band exit only after timeout met") {
    ExitConditions ex({
        .small_error            = 1.0,
        .small_error_timeout_ms = 100.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 500.0,
    });

    ex.feed(0.5, 50.0);
    CHECK_FALSE(ex.is_done());
    ex.feed(0.5, 49.0);
    CHECK_FALSE(ex.is_done());
    ex.feed(0.5, 1.0);
    CHECK(ex.is_done());
}

TEST_CASE("Large-band exit triggers when stuck near edge") {
    ExitConditions ex({
        .small_error            = 0.5,
        .small_error_timeout_ms = 100.0,
        .large_error            = 3.0,
        .large_error_timeout_ms = 200.0,
    });

    for (int i = 0; i < 20; ++i) {
        ex.feed(2.5, 10.0);
    }
    CHECK(ex.is_done());
}

TEST_CASE("Absolute timeout is hard floor") {
    ExitConditions ex({
        .small_error            = 0.0,
        .small_error_timeout_ms = 1e9,
        .large_error            = 0.0,
        .large_error_timeout_ms = 1e9,
        .absolute_timeout_ms    = 500.0,
    });

    for (int i = 0; i < 49; ++i) {
        ex.feed(100.0, 10.0);
    }
    CHECK_FALSE(ex.is_done());
    ex.feed(100.0, 10.0);
    CHECK(ex.is_done());
}

TEST_CASE("Error briefly leaving small band resets accumulator") {
    ExitConditions ex({
        .small_error            = 1.0,
        .small_error_timeout_ms = 100.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 1e9,
    });

    ex.feed(0.5, 60.0);
    ex.feed(2.0, 5.0);
    ex.feed(0.5, 60.0);
    CHECK_FALSE(ex.is_done());
    ex.feed(0.5, 50.0);
    CHECK(ex.is_done());
}

TEST_CASE("Large-band accumulator also resets when error spikes out") {
    ExitConditions ex({
        .small_error            = 0.1,
        .small_error_timeout_ms = 1e9,
        .large_error            = 2.0,
        .large_error_timeout_ms = 100.0,
    });

    ex.feed(1.5, 90.0);
    ex.feed(5.0, 5.0);
    ex.feed(1.5, 50.0);
    CHECK_FALSE(ex.is_done());
}

TEST_CASE("reset() zeros all accumulators") {
    ExitConditions ex({
        .small_error            = 1.0,
        .small_error_timeout_ms = 100.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 200.0,
        .absolute_timeout_ms    = 1000.0,
    });

    for (int i = 0; i < 50; ++i) {
        ex.feed(0.5, 10.0);
    }
    CHECK(ex.is_done());

    ex.reset();
    CHECK_FALSE(ex.is_done());
    CHECK(ex.small_band_held_ms() == doctest::Approx(0.0));
    CHECK(ex.large_band_held_ms() == doctest::Approx(0.0));
    CHECK(ex.total_elapsed_ms()   == doctest::Approx(0.0));
}

TEST_CASE("Zero timeout never triggers that band") {
    ExitConditions ex({
        .small_error            = 1.0,
        .small_error_timeout_ms = 0.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 0.0,
        .absolute_timeout_ms    = 1000.0,
    });

    for (int i = 0; i < 50; ++i) {
        ex.feed(0.0, 10.0);
    }
    CHECK_FALSE(ex.is_done());
}

TEST_CASE("Boundary: exactly equal to small_error counts as in-band") {
    ExitConditions ex({
        .small_error            = 1.0,
        .small_error_timeout_ms = 50.0,
        .large_error            = 5.0,
        .large_error_timeout_ms = 1e9,
    });

    ex.feed(1.0, 30.0);
    ex.feed(1.0, 25.0);
    CHECK(ex.is_done());
}
