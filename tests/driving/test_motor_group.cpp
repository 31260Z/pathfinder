#include <doctest/doctest.h>

#include <pathfinder/driving/mocks.hpp>
#include <pathfinder/driving/motor_group.hpp>

#include <memory>
#include <vector>

using namespace pathfinder;

namespace {

std::shared_ptr<FakeMotor> make_motor(double pos = 0.0, double vel = 0.0,
                                      bool reversed = false) {
    auto m = std::make_shared<FakeMotor>();
    m->set_position(pos);
    m->set_velocity(vel);
    m->set_reversed(reversed);
    return m;
}

} // namespace

TEST_CASE("MotorGroup: empty group is well-defined") {
    MotorGroup g({});
    CHECK(g.size() == 0);
    CHECK(g.average_position_revolutions() == doctest::Approx(0.0));
    CHECK(g.average_velocity_rpm()         == doctest::Approx(0.0));

    // No motors → set_voltage / set_brake are no-ops, must not crash.
    g.set_voltage_mv(1234.0);
    g.set_brake_mode(BrakeMode::Hold);
    g.reset_positions(5.0);
}

TEST_CASE("MotorGroup: initializer-list ctor") {
    auto a = make_motor();
    auto b = make_motor();
    MotorGroup g{a, b};
    CHECK(g.size() == 2);
}

TEST_CASE("MotorGroup: vector ctor") {
    std::vector<std::shared_ptr<IMotor>> v{ make_motor(), make_motor(), make_motor() };
    MotorGroup g(v);
    CHECK(g.size() == 3);
}

TEST_CASE("MotorGroup: set_voltage_mv propagates to every motor") {
    auto a = make_motor();
    auto b = make_motor();
    auto c = make_motor();
    MotorGroup g{a, b, c};

    g.set_voltage_mv(6000.0);
    CHECK(a->last_voltage_mv() == doctest::Approx(6000.0));
    CHECK(b->last_voltage_mv() == doctest::Approx(6000.0));
    CHECK(c->last_voltage_mv() == doctest::Approx(6000.0));

    g.set_voltage_mv(-12000.0);
    CHECK(a->last_voltage_mv() == doctest::Approx(-12000.0));
    CHECK(b->last_voltage_mv() == doctest::Approx(-12000.0));
    CHECK(c->last_voltage_mv() == doctest::Approx(-12000.0));
}

TEST_CASE("MotorGroup: set_brake_mode propagates to every motor") {
    auto a = make_motor();
    auto b = make_motor();
    MotorGroup g{a, b};

    g.set_brake_mode(BrakeMode::Hold);
    CHECK(a->brake_mode() == BrakeMode::Hold);
    CHECK(b->brake_mode() == BrakeMode::Hold);

    g.set_brake_mode(BrakeMode::Coast);
    CHECK(a->brake_mode() == BrakeMode::Coast);
    CHECK(b->brake_mode() == BrakeMode::Coast);
}

TEST_CASE("MotorGroup: average_position_revolutions averages the readings") {
    auto a = make_motor(2.0);
    auto b = make_motor(4.0);
    auto c = make_motor(6.0);
    MotorGroup g{a, b, c};
    CHECK(g.average_position_revolutions() == doctest::Approx(4.0));
}

TEST_CASE("MotorGroup: average_velocity_rpm averages the readings") {
    auto a = make_motor(0.0, 100.0);
    auto b = make_motor(0.0, 200.0);
    MotorGroup g{a, b};
    CHECK(g.average_velocity_rpm() == doctest::Approx(150.0));
}

TEST_CASE("MotorGroup: reversed motors net out correctly when paired") {
    // Two motors mounted side-by-side: one normal at +3 rev, one upside-down
    // marked reversed reading -3 (which the IMotor reverses sign on, so it
    // returns +3). Both motors are physically rotating the wheel forward by
    // 3 revs; the average is +3.
    auto normal = make_motor(3.0);                // returns +3
    auto upside = make_motor(-3.0, 0.0, true);    // returns -(-3) = +3
    MotorGroup g{normal, upside};
    CHECK(g.average_position_revolutions() == doctest::Approx(3.0));
}

TEST_CASE("MotorGroup: reset_positions clears every motor") {
    auto a = make_motor(7.0);
    auto b = make_motor(11.0);
    MotorGroup g{a, b};

    g.reset_positions();
    CHECK(g.average_position_revolutions() == doctest::Approx(0.0));

    g.reset_positions(5.0);
    CHECK(g.average_position_revolutions() == doctest::Approx(5.0));
}

TEST_CASE("MotorGroup: motors() exposes underlying vector for inspection") {
    auto a = make_motor();
    auto b = make_motor();
    MotorGroup g{a, b};
    CHECK(g.motors().size() == 2);
    CHECK(g.motors()[0].get() == a.get());
    CHECK(g.motors()[1].get() == b.get());
}
