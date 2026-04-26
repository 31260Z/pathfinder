#include <doctest/doctest.h>

#include <pathfinder/sensors/distance_sensor.hpp>
#include <pathfinder/sensors/mocks.hpp>

#include <memory>

using namespace pathfinder;

TEST_CASE("DistanceSensor: defaults are sensible") {
    DistanceSensor d;
    CHECK(d.theta_deg    == doctest::Approx(0.0));
    CHECK(d.max_range_in == doctest::Approx(79.0));
    CHECK(d.sigma_in     == doctest::Approx(0.5));
    CHECK(d.offset_z_in  == doctest::Approx(0.0));
    CHECK(d.offset_xy_in.x == doctest::Approx(0.0));
    CHECK(d.offset_xy_in.y == doctest::Approx(0.0));
}

TEST_CASE("DistanceSensor: designated initializers populate fields") {
    auto fake = std::make_shared<FakeDistance>();
    fake->set_distance(12.5);

    DistanceSensor d{
        .sensor       = fake,
        .offset_xy_in = {18.0, 0.0},
        .offset_z_in  = 3.0,
        .theta_deg    = 0.0,
        .max_range_in = 79.0,
        .sigma_in     = 0.5,
    };

    CHECK(d.offset_xy_in.x == doctest::Approx(18.0));
    CHECK(d.offset_z_in    == doctest::Approx(3.0));
    CHECK(d.sensor->distance_in() == doctest::Approx(12.5));
}

TEST_CASE("FakeDistance: valid by default, distance reads through") {
    auto fake = std::make_shared<FakeDistance>();
    fake->set_distance(8.0);
    CHECK(fake->is_valid()    == true);
    CHECK(fake->distance_in() == doctest::Approx(8.0));
}

TEST_CASE("FakeDistance: is_valid can be cleared (no object detected)") {
    auto fake = std::make_shared<FakeDistance>();
    fake->set_distance(50.0);
    fake->set_valid(false);
    CHECK(fake->is_valid() == false);
    // distance_in() still reads — consumers must consult is_valid() first.
    CHECK(fake->distance_in() == doctest::Approx(50.0));
}

TEST_CASE("DistanceSensor: theta_deg sign convention") {
    // 0° = +X (forward); +90° = +Y (right). No math here — just locking the
    // contract via doc comment in the header.
    DistanceSensor forward{ .theta_deg = 0.0 };
    DistanceSensor right  { .theta_deg = 90.0 };
    DistanceSensor back   { .theta_deg = 180.0 };
    CHECK(forward.theta_deg == doctest::Approx(0.0));
    CHECK(right.theta_deg   == doctest::Approx(90.0));
    CHECK(back.theta_deg    == doctest::Approx(180.0));
}
