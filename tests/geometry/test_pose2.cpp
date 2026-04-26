#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

using namespace pathfinder;

namespace {
constexpr double k_pi_local = 3.14159265358979323846;
constexpr double k_test_eps = 1e-9;
} // namespace

TEST_CASE("Pose2: default construction") {
    Pose2 p;
    CHECK(p.x == 0.0);
    CHECK(p.y == 0.0);
    CHECK(p.heading.radians() == 0.0);
}

TEST_CASE("Pose2: identity transform leaves point unchanged") {
    Pose2 id;
    Vector2 v{3.0, 4.0};
    CHECK(id.transform(v) == v);
    CHECK(id.inverse_transform(v) == v);
}

TEST_CASE("Pose2: transform of (0, 0) returns translation") {
    Pose2 p{2.0, 3.0, Angle::degrees(45.0)};
    Vector2 r = p.transform(Vector2{0, 0});
    CHECK(r.x == doctest::Approx(2.0));
    CHECK(r.y == doctest::Approx(3.0));
}

TEST_CASE("Pose2: heading rotation direction (spec correctness check)") {
    // Pose at (0, 0, pi/2): the local +X axis should map to the parent +Y.
    Pose2 p{0.0, 0.0, Angle::radians(k_pi_local / 2.0)};
    Vector2 r = p.transform(Vector2{1, 0});
    CHECK(r.x == doctest::Approx(0.0).epsilon(k_test_eps));
    CHECK(r.y == doctest::Approx(1.0));
}

TEST_CASE("Pose2: inverse_transform of pose translation returns origin") {
    Pose2 p{5.0, -3.0, Angle::degrees(37.0)};
    Vector2 r = p.inverse_transform(Vector2{p.x, p.y});
    CHECK(r.x == doctest::Approx(0.0).epsilon(k_test_eps));
    CHECK(r.y == doctest::Approx(0.0).epsilon(k_test_eps));
}

TEST_CASE("Pose2: transform/inverse_transform round trip") {
    Pose2 p{1.5, 2.5, Angle::degrees(60.0)};
    Vector2 v{4.0, -2.0};
    Vector2 round_trip = p.inverse_transform(p.transform(v));
    CHECK(round_trip.x == doctest::Approx(v.x));
    CHECK(round_trip.y == doctest::Approx(v.y));
}

TEST_CASE("Pose2: inverse cancels compose") {
    Pose2 p{2.0, 3.0, Angle::degrees(30.0)};
    Pose2 round = p.compose(p.inverse());
    Vector2 t = round.transform(Vector2{0, 0});
    CHECK(t.x == doctest::Approx(0.0).epsilon(k_test_eps));
    CHECK(t.y == doctest::Approx(0.0).epsilon(k_test_eps));
    CHECK(round.heading.normalize_signed().radians()
          == doctest::Approx(0.0).epsilon(k_test_eps));
}

TEST_CASE("Pose2: world.compose(local).transform(p) == world.transform(local.transform(p))") {
    Pose2 world{1.0, 2.0, Angle::degrees(20.0)};
    Pose2 local{0.5, -1.0, Angle::degrees(15.0)};
    Vector2 p{3.0, 4.0};

    Vector2 a = world.compose(local).transform(p);
    Vector2 b = world.transform(local.transform(p));

    CHECK(a.x == doctest::Approx(b.x));
    CHECK(a.y == doctest::Approx(b.y));
}

TEST_CASE("Pose2: composition is associative") {
    Pose2 a{1.0, 0.0, Angle::degrees(10.0)};
    Pose2 b{0.0, 1.0, Angle::degrees(20.0)};
    Pose2 c{2.0, -1.0, Angle::degrees(-15.0)};

    Pose2 ab_c = a.compose(b).compose(c);
    Pose2 a_bc = a.compose(b.compose(c));

    Vector2 p{0.5, 0.5};
    Vector2 v1 = ab_c.transform(p);
    Vector2 v2 = a_bc.transform(p);
    CHECK(v1.x == doctest::Approx(v2.x));
    CHECK(v1.y == doctest::Approx(v2.y));
    CHECK(ab_c.heading.normalize_signed().radians()
          == doctest::Approx(a_bc.heading.normalize_signed().radians()));
}

TEST_CASE("Pose2: equality with epsilon") {
    Pose2 a{1.0, 2.0, Angle::degrees(30.0)};
    Pose2 b{1.0 + 1e-12, 2.0, Angle::degrees(30.0)};
    CHECK(a == b);
    Pose2 c{1.5, 2.0, Angle::degrees(30.0)};
    CHECK(a != c);
}

TEST_CASE("Wheel-finder scenario via Pose2: bot pivots about (0,0) by pi/2") {
    // Bot at world origin, oriented +X. Wheel mounted at bot-frame (0, 1).
    // After bot rotates pi/2 CCW (in standard math sense), the wheel's
    // world-frame position is rotate((0,1), pi/2) = (-1, 0).
    Pose2 before{0.0, 0.0, Angle::radians(0.0)};
    Pose2 after{0.0, 0.0, Angle::radians(k_pi_local / 2.0)};

    Vector2 wheel_local{0.0, 1.0};
    Vector2 wheel_before = before.transform(wheel_local);
    Vector2 wheel_after = after.transform(wheel_local);

    CHECK(wheel_before.x == doctest::Approx(0.0));
    CHECK(wheel_before.y == doctest::Approx(1.0));
    CHECK(wheel_after.x == doctest::Approx(-1.0));
    CHECK(wheel_after.y == doctest::Approx(0.0).epsilon(k_test_eps));
}
