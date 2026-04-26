#include <doctest/doctest.h>

#include <pathfinder/sensors/mocks.hpp>
#include <pathfinder/sensors/tracking_wheel.hpp>
#include <pathfinder/sensors/wheel_specs.hpp>

#include <cmath>
#include <memory>

using namespace pathfinder;

namespace {
constexpr double k_pi_local = 3.14159265358979323846;
} // namespace

TEST_CASE("wheel_diameter_in: standard wheels") {
    CHECK(wheel_diameter_in(Wheel::Omni_275)     == doctest::Approx(2.75));
    CHECK(wheel_diameter_in(Wheel::Omni_325)     == doctest::Approx(3.25));
    CHECK(wheel_diameter_in(Wheel::Omni_4)       == doctest::Approx(4.0));
    CHECK(wheel_diameter_in(Wheel::Traction_275) == doctest::Approx(2.75));
    CHECK(wheel_diameter_in(Wheel::Traction_4)   == doctest::Approx(4.0));
}

TEST_CASE("wheel_diameter_in: Wheel::Custom is a programmer error") {
    CHECK_THROWS_AS(wheel_diameter_in(Wheel::Custom), std::invalid_argument);
}

TEST_CASE("TrackingWheel::diameter_in resolves Custom to custom_diameter_in") {
    TrackingWheel w;
    w.wheel              = Wheel::Custom;
    w.custom_diameter_in = 1.625;
    CHECK(w.diameter_in() == doctest::Approx(1.625));
}

TEST_CASE("TrackingWheel::diameter_in returns standard for non-Custom") {
    TrackingWheel w;
    w.wheel = Wheel::Omni_325;
    CHECK(w.diameter_in() == doctest::Approx(3.25));
}

TEST_CASE("TrackingWheel::encoder_to_inches: 1 rev of 2.75 omni = π·2.75 in") {
    TrackingWheel w;
    w.wheel = Wheel::Omni_275;
    CHECK(w.encoder_to_inches(1.0) == doctest::Approx(k_pi_local * 2.75));
}

TEST_CASE("TrackingWheel::encoder_to_inches: respects gear_ratio") {
    TrackingWheel w;
    w.wheel      = Wheel::Omni_4;
    w.gear_ratio = 0.5;   // wheel turns half as fast as encoder
    CHECK(w.encoder_to_inches(2.0) == doctest::Approx(k_pi_local * 4.0 * 1.0));
}

TEST_CASE("TrackingWheel::encoder_to_inches: signed input maps signed output") {
    TrackingWheel w;
    w.wheel = Wheel::Omni_325;
    CHECK(w.encoder_to_inches(-1.0) == doctest::Approx(-k_pi_local * 3.25));
}

TEST_CASE("FakeRotation: reverse flag flips position and velocity sign") {
    auto fake = std::make_shared<FakeRotation>();
    fake->set_position(2.0);
    fake->set_velocity(5.0);
    CHECK(fake->position_revolutions() == doctest::Approx(2.0));
    CHECK(fake->velocity_rps()         == doctest::Approx(5.0));
    CHECK(fake->is_reversed() == false);

    fake->set_reversed(true);
    CHECK(fake->position_revolutions() == doctest::Approx(-2.0));
    CHECK(fake->velocity_rps()         == doctest::Approx(-5.0));
    CHECK(fake->is_reversed() == true);
}

TEST_CASE("FakeRotation: reset_position sets the underlying value") {
    auto fake = std::make_shared<FakeRotation>();
    fake->set_position(7.0);
    fake->reset_position(0.0);
    CHECK(fake->position_revolutions() == doctest::Approx(0.0));
    fake->reset_position(3.5);
    CHECK(fake->position_revolutions() == doctest::Approx(3.5));
}

TEST_CASE("TrackingWheel: full pipeline — fake encoder revs through to inches") {
    auto fake = std::make_shared<FakeRotation>();
    TrackingWheel w;
    w.sensor = fake;
    w.wheel  = Wheel::Omni_275;

    fake->set_position(4.0);
    const double inches = w.encoder_to_inches(w.sensor->position_revolutions());
    CHECK(inches == doctest::Approx(4.0 * k_pi_local * 2.75));
}

TEST_CASE("TrackingWheel: reversed sensor reverses inches reading") {
    auto fake = std::make_shared<FakeRotation>();
    fake->set_position(1.0);
    fake->set_reversed(true);

    TrackingWheel w;
    w.sensor = fake;
    w.wheel  = Wheel::Omni_4;

    const double inches = w.encoder_to_inches(w.sensor->position_revolutions());
    CHECK(inches == doctest::Approx(-1.0 * k_pi_local * 4.0));
}
