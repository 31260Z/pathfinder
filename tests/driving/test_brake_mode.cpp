#include <doctest/doctest.h>

#include <pathfinder/driving/interfaces.hpp>
#include <pathfinder/driving/mocks.hpp>

using namespace pathfinder;

TEST_CASE("BrakeMode: enum values are distinct") {
    CHECK(BrakeMode::Coast != BrakeMode::Brake);
    CHECK(BrakeMode::Brake != BrakeMode::Hold);
    CHECK(BrakeMode::Coast != BrakeMode::Hold);
}

TEST_CASE("FakeMotor: defaults to Coast brake mode") {
    FakeMotor m;
    CHECK(m.brake_mode() == BrakeMode::Coast);
}

TEST_CASE("FakeMotor: set_brake_mode round-trips") {
    FakeMotor m;
    m.set_brake_mode(BrakeMode::Hold);
    CHECK(m.brake_mode() == BrakeMode::Hold);
    m.set_brake_mode(BrakeMode::Brake);
    CHECK(m.brake_mode() == BrakeMode::Brake);
    m.set_brake_mode(BrakeMode::Coast);
    CHECK(m.brake_mode() == BrakeMode::Coast);
}

TEST_CASE("FakeMotor: set_voltage_mv records last value, no clamping") {
    FakeMotor m;
    m.set_voltage_mv(0.0);
    CHECK(m.last_voltage_mv() == doctest::Approx(0.0));
    m.set_voltage_mv(7777.0);
    CHECK(m.last_voltage_mv() == doctest::Approx(7777.0));
    // Clamping is the adapter's job, not the abstraction's; the fake stores
    // whatever it was handed so tests can verify exactly what the chassis
    // commanded.
    m.set_voltage_mv(99999.0);
    CHECK(m.last_voltage_mv() == doctest::Approx(99999.0));
}
