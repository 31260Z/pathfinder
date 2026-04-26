#include <doctest/doctest.h>

#include <pathfinder/geometry/vector2.hpp>

#include <cmath>

using namespace pathfinder;

namespace {
constexpr double k_pi = 3.14159265358979323846;
constexpr double k_test_eps = 1e-9;
} // namespace

TEST_CASE("Vector2: default construction is zero") {
    Vector2 v;
    CHECK(v.x == 0.0);
    CHECK(v.y == 0.0);
}

TEST_CASE("Vector2: arithmetic operators") {
    Vector2 a{1.0, 2.0};
    Vector2 b{3.0, 4.0};

    CHECK((a + b) == Vector2{4.0, 6.0});
    CHECK((a - b) == Vector2{-2.0, -2.0});
    CHECK((-a) == Vector2{-1.0, -2.0});
    CHECK((a * 2.0) == Vector2{2.0, 4.0});
    CHECK((2.0 * a) == Vector2{2.0, 4.0});
    CHECK((a / 2.0) == Vector2{0.5, 1.0});
}

TEST_CASE("Vector2: compound assignment") {
    Vector2 v{1.0, 2.0};
    v += Vector2{1.0, 1.0};
    CHECK(v == Vector2{2.0, 3.0});
    v -= Vector2{1.0, 1.0};
    CHECK(v == Vector2{1.0, 2.0});
    v *= 2.0;
    CHECK(v == Vector2{2.0, 4.0});
    v /= 2.0;
    CHECK(v == Vector2{1.0, 2.0});
}

TEST_CASE("Vector2: equality with epsilon") {
    Vector2 a{1.0, 2.0};
    Vector2 b{1.0 + 1e-12, 2.0 - 1e-12};
    CHECK(a == b);
    CHECK_FALSE(a != b);
    CHECK(a != Vector2{1.1, 2.0});
}

TEST_CASE("Vector2: dot product") {
    CHECK(dot(Vector2{1, 0}, Vector2{0, 1}) == 0.0);
    CHECK(dot(Vector2{1, 0}, Vector2{1, 0}) == 1.0);
    CHECK(dot(Vector2{2, 3}, Vector2{4, 5}) == doctest::Approx(23.0));
}

TEST_CASE("Vector2: cross product (scalar 2D form)") {
    CHECK(cross(Vector2{1, 0}, Vector2{0, 1}) == 1.0);
    CHECK(cross(Vector2{0, 1}, Vector2{1, 0}) == -1.0);
    CHECK(cross(Vector2{1, 0}, Vector2{1, 0}) == 0.0);
    CHECK(cross(Vector2{2, 3}, Vector2{4, 5}) == doctest::Approx(-2.0));
}

TEST_CASE("Vector2: perp is 90deg CCW rotation") {
    CHECK(perp(Vector2{1, 0}) == Vector2{0, 1});
    CHECK(perp(Vector2{0, 1}) == Vector2{-1, 0});
    CHECK(perp(Vector2{-1, 0}) == Vector2{0, -1});
    CHECK(perp(Vector2{0, -1}) == Vector2{1, 0});
}

TEST_CASE("Vector2: perp identity dot(v, perp(v)) == 0") {
    Vector2 v{3.5, -2.1};
    CHECK(dot(v, perp(v)) == doctest::Approx(0.0));
}

TEST_CASE("Vector2: norm and norm_sq") {
    Vector2 v{3.0, 4.0};
    CHECK(norm_sq(v) == 25.0);
    CHECK(norm(v) == doctest::Approx(5.0));
    CHECK(norm(Vector2{0, 0}) == 0.0);
}

TEST_CASE("Vector2: normalize unit vector") {
    Vector2 v{3.0, 4.0};
    Vector2 u = normalize(v);
    CHECK(norm(u) == doctest::Approx(1.0));
    CHECK(u.x == doctest::Approx(0.6));
    CHECK(u.y == doctest::Approx(0.8));
}

TEST_CASE("Vector2: normalize zero throws") {
    CHECK_THROWS_AS(normalize(Vector2{0, 0}), std::domain_error);
}

TEST_CASE("Vector2: rotate 90deg CCW maps +X to +Y") {
    Vector2 r = rotate(Vector2{1, 0}, k_pi / 2.0);
    CHECK(r.x == doctest::Approx(0.0).epsilon(k_test_eps));
    CHECK(r.y == doctest::Approx(1.0));
}

TEST_CASE("Vector2: rotate is identity for 0 and 2pi") {
    Vector2 v{1.5, -2.7};
    Vector2 r0 = rotate(v, 0.0);
    Vector2 r_2pi = rotate(v, 2.0 * k_pi);
    CHECK(r0 == v);
    CHECK(r_2pi == v);
}

TEST_CASE("Vector2: rotate by pi negates") {
    Vector2 v{2.0, 3.0};
    Vector2 r = rotate(v, k_pi);
    CHECK(r.x == doctest::Approx(-2.0));
    CHECK(r.y == doctest::Approx(-3.0));
}

TEST_CASE("Vector2: rotate by pi/2 equals perp") {
    Vector2 v{2.0, 3.0};
    Vector2 r = rotate(v, k_pi / 2.0);
    Vector2 p = perp(v);
    CHECK(r.x == doctest::Approx(p.x));
    CHECK(r.y == doctest::Approx(p.y));
}

TEST_CASE("Vector2: distance") {
    CHECK(distance(Vector2{0, 0}, Vector2{3, 4}) == doctest::Approx(5.0));
    CHECK(distance(Vector2{1, 1}, Vector2{1, 1}) == 0.0);
}

TEST_CASE("Vector2: angle_to signed") {
    CHECK(angle_to(Vector2{1, 0}, Vector2{0, 1}) == doctest::Approx(k_pi / 2.0));
    CHECK(angle_to(Vector2{1, 0}, Vector2{0, -1}) == doctest::Approx(-k_pi / 2.0));
    CHECK(angle_to(Vector2{1, 0}, Vector2{1, 0}) == doctest::Approx(0.0));
    CHECK(std::abs(angle_to(Vector2{1, 0}, Vector2{-1, 0})) == doctest::Approx(k_pi));
}

TEST_CASE("Wheel-finder math: parallel wheel encoder reads (P_y - y_p) * dtheta") {
    // Spec Appendix A: bot rotates by dtheta about pivot P; wheel at Q in bot
    // frame; displacement is dtheta * perp(Q - P), encoder is along (1, 0).
    Vector2 P{0.0, 0.0};
    Vector2 Q{0.0, 1.0};   // wheel 1" right of pivot
    double dtheta = k_pi / 2.0;

    Vector2 disp = dtheta * perp(Q - P);
    double encoder = dot(disp, Vector2{1, 0});

    // Expected reading: (P_y - y_p) * dtheta = (0 - 1) * pi/2 = -pi/2
    CHECK(encoder == doctest::Approx(-k_pi / 2.0));

    // Direction sanity: wheel at +Y of pivot, bot rotates CCW (positive dtheta),
    // so the wheel sweeps backward (-X) — encoder reads negative.
    CHECK(disp.x == doctest::Approx(-k_pi / 2.0));
    CHECK(disp.y == doctest::Approx(0.0).epsilon(k_test_eps));
}

TEST_CASE("Wheel-finder math: perpendicular wheel reads (x_q - P_x) * dtheta") {
    Vector2 P{0.0, 0.0};
    Vector2 Q{2.0, 0.0};
    double dtheta = 0.5;

    Vector2 disp = dtheta * perp(Q - P);
    double encoder = dot(disp, Vector2{0, 1});

    CHECK(encoder == doctest::Approx((Q.x - P.x) * dtheta));
}
