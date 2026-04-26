#include <doctest/doctest.h>

#include <pathfinder/geometry/params.hpp>
#include <pathfinder/geometry/vector2.hpp>

using namespace pathfinder;

TEST_CASE("params: forward = a, right = b -> (a, b)") {
    CHECK(resolve_point(forward = 5.0, right = 10.0) == Vector2{5.0, 10.0});
}

TEST_CASE("params: forward = a, left = b -> (a, -b)") {
    CHECK(resolve_point(forward = 5.0, left = 10.0) == Vector2{5.0, -10.0});
}

TEST_CASE("params: backward = a, right = b -> (-a, b)") {
    CHECK(resolve_point(backward = 5.0, right = 10.0) == Vector2{-5.0, 10.0});
}

TEST_CASE("params: backward = a, left = b -> (-a, -b)") {
    CHECK(resolve_point(backward = 5.0, left = 10.0) == Vector2{-5.0, -10.0});
}

TEST_CASE("params: x_ = a, y_ = b -> (a, b) raw passthrough") {
    CHECK(resolve_point(x_ = 3.0, y_ = -4.0) == Vector2{3.0, -4.0});
    CHECK(resolve_point(x_ = -7.0, y_ = 0.0) == Vector2{-7.0, 0.0});
}

TEST_CASE("params: order doesn't matter") {
    CHECK(resolve_point(right = 10.0, forward = 5.0) == Vector2{5.0, 10.0});
    CHECK(resolve_point(left = 4.0, backward = 2.0) == Vector2{-2.0, -4.0});
}

TEST_CASE("params: mixing x-axis with y-axis works in either combo") {
    CHECK(resolve_point(forward = 1.0, y_ = 2.0) == Vector2{1.0, 2.0});
    CHECK(resolve_point(x_ = 1.0, right = 2.0) == Vector2{1.0, 2.0});
    CHECK(resolve_point(backward = 1.0, y_ = 2.0) == Vector2{-1.0, 2.0});
}

TEST_CASE("params: missing y-axis throws at runtime") {
    CHECK_THROWS_AS(resolve_point(forward = 5.0), std::invalid_argument);
}

TEST_CASE("params: missing x-axis throws at runtime") {
    CHECK_THROWS_AS(resolve_point(right = 5.0), std::invalid_argument);
}

TEST_CASE("params: zero-arg call throws at runtime") {
    CHECK_THROWS_AS(resolve_point(), std::invalid_argument);
}

// Compile-time conflict checks. These cannot be invoked at runtime because
// the static_assert fires; they're documented here to record the contract.
//
//   resolve_point(forward = 1.0, backward = 2.0, right = 3.0)  // static_assert
//   resolve_point(left = 1.0, right = 2.0, forward = 3.0)      // static_assert
//   resolve_point(forward = 1.0, x_ = 2.0, right = 3.0)        // static_assert
//   resolve_point(left = 1.0, y_ = 2.0, forward = 3.0)         // static_assert
