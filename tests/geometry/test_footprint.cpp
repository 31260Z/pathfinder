#include <doctest/doctest.h>

#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/geometry/vector2.hpp>

using namespace pathfinder;

TEST_CASE("Footprint: BackLeft origin corner positions") {
    Footprint fp{18.0, 18.0};
    auto c = fp.corners(Corner::BackLeft);
    // Order: FrontLeft, FrontRight, BackLeft, BackRight
    CHECK(c[0] == Vector2{18.0, 0.0});    // FrontLeft
    CHECK(c[1] == Vector2{18.0, 18.0});   // FrontRight
    CHECK(c[2] == Vector2{0.0, 0.0});     // BackLeft
    CHECK(c[3] == Vector2{0.0, 18.0});    // BackRight
}

TEST_CASE("Footprint: BackRight origin corner positions") {
    Footprint fp{18.0, 18.0};
    auto c = fp.corners(Corner::BackRight);
    CHECK(c[0] == Vector2{18.0, -18.0});  // FrontLeft
    CHECK(c[1] == Vector2{18.0, 0.0});    // FrontRight
    CHECK(c[2] == Vector2{0.0, -18.0});   // BackLeft
    CHECK(c[3] == Vector2{0.0, 0.0});     // BackRight
}

TEST_CASE("Footprint: FrontLeft origin corner positions") {
    Footprint fp{18.0, 18.0};
    auto c = fp.corners(Corner::FrontLeft);
    CHECK(c[0] == Vector2{0.0, 0.0});      // FrontLeft
    CHECK(c[1] == Vector2{0.0, 18.0});     // FrontRight
    CHECK(c[2] == Vector2{-18.0, 0.0});    // BackLeft
    CHECK(c[3] == Vector2{-18.0, 18.0});   // BackRight
}

TEST_CASE("Footprint: FrontRight origin corner positions") {
    Footprint fp{18.0, 18.0};
    auto c = fp.corners(Corner::FrontRight);
    CHECK(c[0] == Vector2{0.0, -18.0});    // FrontLeft
    CHECK(c[1] == Vector2{0.0, 0.0});      // FrontRight
    CHECK(c[2] == Vector2{-18.0, -18.0});  // BackLeft
    CHECK(c[3] == Vector2{-18.0, 0.0});    // BackRight
}

TEST_CASE("Footprint: rectangular (non-square) BackLeft") {
    Footprint fp{20.0, 12.0};
    auto c = fp.corners(Corner::BackLeft);
    CHECK(c[0] == Vector2{20.0, 0.0});
    CHECK(c[1] == Vector2{20.0, 12.0});
    CHECK(c[2] == Vector2{0.0, 0.0});
    CHECK(c[3] == Vector2{0.0, 12.0});
}

TEST_CASE("Footprint: inflate") {
    Footprint fp{18.0, 12.0};
    Footprint big = fp.inflate(2.0);
    CHECK(big.length == doctest::Approx(22.0));
    CHECK(big.width == doctest::Approx(16.0));
}

TEST_CASE("Footprint: inflate by zero is no-op") {
    Footprint fp{18.0, 12.0};
    Footprint same = fp.inflate(0.0);
    CHECK(same.length == fp.length);
    CHECK(same.width == fp.width);
}

TEST_CASE("Footprint: contains for BackLeft") {
    Footprint fp{18.0, 18.0};
    CHECK(fp.contains(Vector2{0, 0}, Corner::BackLeft));
    CHECK(fp.contains(Vector2{18, 18}, Corner::BackLeft));
    CHECK(fp.contains(Vector2{9, 9}, Corner::BackLeft));
    CHECK_FALSE(fp.contains(Vector2{-1, 9}, Corner::BackLeft));
    CHECK_FALSE(fp.contains(Vector2{19, 9}, Corner::BackLeft));
    CHECK_FALSE(fp.contains(Vector2{9, -1}, Corner::BackLeft));
    CHECK_FALSE(fp.contains(Vector2{9, 19}, Corner::BackLeft));
}

TEST_CASE("Footprint: contains for FrontRight") {
    Footprint fp{18.0, 18.0};
    CHECK(fp.contains(Vector2{0, 0}, Corner::FrontRight));
    CHECK(fp.contains(Vector2{-18, -18}, Corner::FrontRight));
    CHECK(fp.contains(Vector2{-9, -9}, Corner::FrontRight));
    CHECK_FALSE(fp.contains(Vector2{1, 0}, Corner::FrontRight));
    CHECK_FALSE(fp.contains(Vector2{0, 1}, Corner::FrontRight));
}
