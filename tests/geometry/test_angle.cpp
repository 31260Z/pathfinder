#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>

#include <cmath>

using namespace pathfinder;

namespace {
constexpr double k_pi_local = 3.14159265358979323846;
} // namespace

TEST_CASE("Angle: default construction is zero radians") {
    Angle a;
    CHECK(a.radians() == 0.0);
    CHECK(a.degrees() == 0.0);
}

TEST_CASE("Angle: degrees<->radians round trip") {
    Angle a = Angle::degrees(90.0);
    CHECK(a.radians() == doctest::Approx(k_pi_local / 2.0));
    CHECK(a.degrees() == doctest::Approx(90.0));

    Angle b = Angle::radians(k_pi_local);
    CHECK(b.degrees() == doctest::Approx(180.0));
    CHECK(b.radians() == doctest::Approx(k_pi_local));
}

TEST_CASE("Angle: degrees factory for negative and large") {
    CHECK(Angle::degrees(-90.0).radians() == doctest::Approx(-k_pi_local / 2.0));
    CHECK(Angle::degrees(720.0).radians() == doctest::Approx(4.0 * k_pi_local));
}

TEST_CASE("Angle: normalize_signed maps to (-pi, pi]") {
    CHECK(Angle::radians(0.0).normalize_signed().radians() == doctest::Approx(0.0));
    CHECK(Angle::radians(k_pi_local).normalize_signed().radians() == doctest::Approx(k_pi_local));
    CHECK(Angle::radians(-k_pi_local).normalize_signed().radians() == doctest::Approx(k_pi_local));
    CHECK(Angle::radians(3.0 * k_pi_local).normalize_signed().radians() == doctest::Approx(k_pi_local));
    CHECK(Angle::radians(-3.0 * k_pi_local).normalize_signed().radians() == doctest::Approx(k_pi_local));
    CHECK(Angle::radians(k_pi_local / 2.0).normalize_signed().radians() == doctest::Approx(k_pi_local / 2.0));
    CHECK(Angle::radians(-k_pi_local / 2.0).normalize_signed().radians() == doctest::Approx(-k_pi_local / 2.0));
    CHECK(Angle::radians(2.0 * k_pi_local + 0.1).normalize_signed().radians() == doctest::Approx(0.1));
}

TEST_CASE("Angle: normalize_unsigned maps to [0, 2pi)") {
    CHECK(Angle::radians(0.0).normalize_unsigned().radians() == doctest::Approx(0.0));
    CHECK(Angle::radians(2.0 * k_pi_local).normalize_unsigned().radians() == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(Angle::radians(-k_pi_local / 2.0).normalize_unsigned().radians() == doctest::Approx(3.0 * k_pi_local / 2.0));
    CHECK(Angle::radians(3.0 * k_pi_local).normalize_unsigned().radians() == doctest::Approx(k_pi_local));
    CHECK(Angle::radians(5.5).normalize_unsigned().radians() == doctest::Approx(5.5));
}

TEST_CASE("Angle: arithmetic") {
    Angle a = Angle::degrees(45.0);
    Angle b = Angle::degrees(30.0);
    CHECK((a + b).degrees() == doctest::Approx(75.0));
    CHECK((a - b).degrees() == doctest::Approx(15.0));
    CHECK((-a).degrees() == doctest::Approx(-45.0));
}

TEST_CASE("Angle: equality with epsilon") {
    Angle a = Angle::radians(1.0);
    Angle b = Angle::radians(1.0 + 1e-12);
    CHECK(a == b);
    CHECK_FALSE(a != b);
    Angle c = Angle::radians(1.1);
    CHECK(a != c);
}

TEST_CASE("shortest_angle: trivial cases") {
    CHECK(shortest_angle(Angle::radians(0.0), Angle::radians(0.0)).radians()
          == doctest::Approx(0.0));
    CHECK(shortest_angle(Angle::degrees(0.0), Angle::degrees(90.0)).degrees()
          == doctest::Approx(90.0));
    CHECK(shortest_angle(Angle::degrees(90.0), Angle::degrees(0.0)).degrees()
          == doctest::Approx(-90.0));
}

TEST_CASE("shortest_angle: wraparound") {
    // 350 -> 10 should be +20, not -340
    CHECK(shortest_angle(Angle::degrees(350.0), Angle::degrees(10.0)).degrees()
          == doctest::Approx(20.0));
    // 10 -> 350 should be -20, not +340
    CHECK(shortest_angle(Angle::degrees(10.0), Angle::degrees(350.0)).degrees()
          == doctest::Approx(-20.0));
    // 0 -> 180: pi (boundary case included)
    CHECK(std::abs(shortest_angle(Angle::degrees(0.0), Angle::degrees(180.0)).degrees())
          == doctest::Approx(180.0));
}
