#include <doctest/doctest.h>

#include <pathfinder/geometry/bot.hpp>
#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/sensors/frame_helpers.hpp>

using namespace pathfinder;

TEST_CASE("center_in_corner_frame: BackLeft origin → center is (+l/2, +w/2)") {
    auto bot = Bot().footprint(18.0, 12.0).origin(Corner::BackLeft);
    const Vector2 c = center_in_corner_frame(bot);
    CHECK(c.x == doctest::Approx(9.0));
    CHECK(c.y == doctest::Approx(6.0));
}

TEST_CASE("center_in_corner_frame: BackRight origin → (+l/2, -w/2)") {
    auto bot = Bot().footprint(18.0, 12.0).origin(Corner::BackRight);
    const Vector2 c = center_in_corner_frame(bot);
    CHECK(c.x == doctest::Approx(9.0));
    CHECK(c.y == doctest::Approx(-6.0));
}

TEST_CASE("center_in_corner_frame: FrontLeft origin → (-l/2, +w/2)") {
    auto bot = Bot().footprint(18.0, 12.0).origin(Corner::FrontLeft);
    const Vector2 c = center_in_corner_frame(bot);
    CHECK(c.x == doctest::Approx(-9.0));
    CHECK(c.y == doctest::Approx(6.0));
}

TEST_CASE("center_in_corner_frame: FrontRight origin → (-l/2, -w/2)") {
    auto bot = Bot().footprint(18.0, 12.0).origin(Corner::FrontRight);
    const Vector2 c = center_in_corner_frame(bot);
    CHECK(c.x == doctest::Approx(-9.0));
    CHECK(c.y == doctest::Approx(-6.0));
}

TEST_CASE("corner_to_center_offset: BackLeft, sensor on bot center reads (0,0)") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::BackLeft);
    // Sensor at corner-frame (9, 9) is the bot center.
    CHECK(corner_to_center_offset({9.0, 9.0}, bot) == Vector2{0.0, 0.0});
}

TEST_CASE("corner_to_center_offset: BackLeft, sensor at origin reads (-9,-9)") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::BackLeft);
    CHECK(corner_to_center_offset({0.0, 0.0}, bot) == Vector2{-9.0, -9.0});
}

TEST_CASE("corner_to_center_offset: BackLeft, generic sensor location") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::BackLeft);
    // Sensor at corner-frame (5, 12) → center-relative (5−9, 12−9) = (−4, 3).
    CHECK(corner_to_center_offset({5.0, 12.0}, bot) == Vector2{-4.0, 3.0});
}

TEST_CASE("corner_to_center_offset: BackRight, sensor at origin reads (-9, +9)") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::BackRight);
    // Center is at (+9, -9). Sensor at (0, 0) → (0-9, 0-(-9)) = (-9, +9).
    CHECK(corner_to_center_offset({0.0, 0.0}, bot) == Vector2{-9.0, 9.0});
}

TEST_CASE("corner_to_center_offset: FrontLeft, sensor at origin reads (+9,-9)") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::FrontLeft);
    // Center is at (-9, +9). Sensor at (0, 0) → (0-(-9), 0-9) = (9, -9).
    CHECK(corner_to_center_offset({0.0, 0.0}, bot) == Vector2{9.0, -9.0});
}

TEST_CASE("corner_to_center_offset: FrontRight, sensor at origin reads (+9,+9)") {
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::FrontRight);
    // Center is at (-9, -9). Sensor at (0, 0) → (0-(-9), 0-(-9)) = (9, 9).
    CHECK(corner_to_center_offset({0.0, 0.0}, bot) == Vector2{9.0, 9.0});
}

TEST_CASE("corner_to_center_offset: non-square footprint") {
    auto bot = Bot().footprint(20.0, 10.0).origin(Corner::BackLeft);
    // Center at (10, 5). Sensor at corner (0, 0) → (-10, -5).
    CHECK(corner_to_center_offset({0.0, 0.0}, bot) == Vector2{-10.0, -5.0});
}

TEST_CASE("corner_to_center_offset: spec §6 example — parallel wheel, BackLeft") {
    // Spec §6 declares a parallel tracking wheel at corner-frame
    // {x: 0, y: 3.5} on a back-left-origin bot. For an 18×18 bot the center
    // sits at (9, 9), so this wheel is at center-relative (−9, −5.5) — i.e.
    // 9" behind center, 5.5" left of center.
    auto bot = Bot().footprint(18.0, 18.0).origin(Corner::BackLeft);
    CHECK(corner_to_center_offset({0.0, 3.5}, bot) == Vector2{-9.0, -5.5});
}
