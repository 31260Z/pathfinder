#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/odometry/field_map.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

using namespace pathfinder;

namespace {
constexpr double k_max_range = 80.0;
} // namespace

TEST_CASE("FieldMap: empty map reports no occupancy anywhere inside") {
    FieldMap m{};   // 144x144 default, all-free
    CHECK_FALSE(m.is_occupied({0.0, 0.0}));
    CHECK_FALSE(m.is_occupied({-50.0, 30.0}));
    CHECK_FALSE(m.is_occupied({71.5, 71.5}));
    // Out-of-bounds points are not occupied.
    CHECK_FALSE(m.is_occupied({100.0, 0.0}));
    CHECK_FALSE(m.is_occupied({-100.0, 0.0}));
}

TEST_CASE("FieldMap: rect obstacle marks expected cells occupied") {
    FieldMap m{};
    // 10"x10" rect centered around (10, 10) — covers x∈[10,20), y∈[10,20).
    m.add_rect_obstacle(10.0, 10.0, 10.0, 10.0);
    CHECK(m.is_occupied({10.5, 10.5}));     // corner
    CHECK(m.is_occupied({15.0, 15.0}));     // center
    CHECK(m.is_occupied({19.5, 19.5}));     // far corner of last full cell
    CHECK_FALSE(m.is_occupied({9.5, 15.0}));   // just outside (-X)
    CHECK_FALSE(m.is_occupied({20.5, 15.0}));  // just outside (+X)
    CHECK_FALSE(m.is_occupied({15.0, 9.5}));   // just outside (-Y)
    CHECK_FALSE(m.is_occupied({15.0, 20.5}));  // just outside (+Y)
}

TEST_CASE("FieldMap: ray-cast hits a rect obstacle at the expected distance") {
    FieldMap m{};
    // Wall covers x ∈ [20, 21), y ∈ [-30, 30). Cast +X from origin.
    m.add_rect_obstacle(20.0, -30.0, 1.0, 60.0);
    const double d = m.cast_ray({0.0, 0.0}, 0.0, k_max_range);
    // Hit cell center is at x=20.5; expect ≈ 20.5 inches.
    CHECK(d == doctest::Approx(20.5).epsilon(0.05));
}

TEST_CASE("FieldMap: ray-cast that misses returns max_range + 1") {
    FieldMap m{};
    const double d = m.cast_ray({0.0, 0.0}, 0.0, 30.0);
    // Empty map, +X ray would exit the field at x=72 — beyond max_range=30.
    CHECK(d == doctest::Approx(31.0));
}

TEST_CASE("FieldMap: ray-cast in arbitrary direction") {
    FieldMap m{};
    // Wall 20" north (+Y) of origin.
    m.add_rect_obstacle(-30.0, 20.0, 60.0, 1.0);
    // Ray pointing +Y (heading = π/2 in our convention; +X→+Y is CW).
    const double d = m.cast_ray({0.0, 0.0}, k_pi / 2.0, k_max_range);
    CHECK(d == doctest::Approx(20.5).epsilon(0.05));
}

TEST_CASE("FieldMap: ray-cast diagonally hits a square obstacle") {
    FieldMap m{};
    // 4x4 square centered at (20, 20).
    m.add_rect_obstacle(18.0, 18.0, 4.0, 4.0);
    // 45° diagonal from origin.
    const double d = m.cast_ray({0.0, 0.0}, k_pi / 4.0, k_max_range);
    // Hit somewhere near (18, 18) → distance ≈ 18·√2 ≈ 25.46.
    CHECK(d == doctest::Approx(18.0 * std::sqrt(2.0)).epsilon(0.10));
}

TEST_CASE("FieldMap: ray starting inside obstacle returns 0") {
    FieldMap m{};
    m.add_rect_obstacle(0.0, 0.0, 10.0, 10.0);
    const double d = m.cast_ray({5.0, 5.0}, 0.0, k_max_range);
    CHECK(d == doctest::Approx(0.0));
}

TEST_CASE("FieldMap: circle obstacle") {
    FieldMap m{};
    m.add_circle_obstacle({30.0, 0.0}, 5.0);
    // Inside the circle.
    CHECK(m.is_occupied({30.5, 0.5}));
    // Just outside.
    CHECK_FALSE(m.is_occupied({36.5, 0.0}));
    // Ray from origin along +X hits roughly at x=25 (circle near edge).
    const double d = m.cast_ray({0.0, 0.0}, 0.0, k_max_range);
    CHECK(d > 24.0);
    CHECK(d < 26.0);
}

TEST_CASE("FieldMap: polygon obstacle") {
    FieldMap m{};
    m.add_polygon_obstacle({
        {10.0, 10.0}, {20.0, 10.0}, {20.0, 20.0}, {10.0, 20.0}
    });
    CHECK(m.is_occupied({15.0, 15.0}));
    CHECK_FALSE(m.is_occupied({5.0, 5.0}));
    CHECK_FALSE(m.is_occupied({25.0, 25.0}));
}

TEST_CASE("FieldMap: default_perimeter has wall cells but interior is free") {
    FieldMap m = FieldMap::default_perimeter();
    // Center is free.
    CHECK_FALSE(m.is_occupied({0.0, 0.0}));
    CHECK_FALSE(m.is_occupied({30.0, 30.0}));
    CHECK_FALSE(m.is_occupied({-50.0, -50.0}));
    // Walls present at boundary cells.
    CHECK(m.is_occupied({-71.75, 0.0}));    // west wall
    CHECK(m.is_occupied({71.75, 0.0}));     // east wall
    CHECK(m.is_occupied({0.0, -71.75}));    // south wall
    CHECK(m.is_occupied({0.0, 71.75}));     // north wall
    // A ray from origin +X should hit at ~ +71.75 (east wall).
    const double d = m.cast_ray({0.0, 0.0}, 0.0, 100.0);
    CHECK(d > 70.0);
    CHECK(d < 73.0);
}

TEST_CASE("FieldMap: JSON load — minimal map with rect, polygon, circle, landmark") {
    const std::string json = R"({
        "name": "test",
        "size_in": [144, 144],
        "origin": "center",
        "obstacles": [
            {"type": "rect",    "x": 10, "y": 10, "w": 5, "h": 5},
            {"type": "polygon", "vertices": [[-30, -30], [-20, -30], [-20, -20], [-30, -20]]},
            {"type": "circle",  "center": [40, 0], "radius": 3}
        ],
        "landmarks": [
            {"name": "tag_1", "x": 60, "y": 0, "heading_deg": 90},
            {"name": "tag_2", "x": -10, "y": 5}
        ]
    })";
    FieldMap m = FieldMap::load_from_json_string(json);

    // Rect verified.
    CHECK(m.is_occupied({12.5, 12.5}));
    CHECK_FALSE(m.is_occupied({0.0, 0.0}));

    // Polygon verified.
    CHECK(m.is_occupied({-25.0, -25.0}));

    // Circle verified.
    CHECK(m.is_occupied({40.0, 0.5}));

    // Landmarks loaded.
    REQUIRE(m.landmarks().size() == 2);
    CHECK(m.landmarks()[0].name == "tag_1");
    CHECK(m.landmarks()[0].field_pose.x == doctest::Approx(60.0));
    CHECK(m.landmarks()[0].field_pose.y == doctest::Approx(0.0));
    CHECK(m.landmarks()[0].field_pose.heading.degrees() == doctest::Approx(90.0));
    CHECK(m.landmarks()[1].name == "tag_2");
    CHECK(m.landmarks()[1].field_pose.heading.degrees() == doctest::Approx(0.0));

    // Origin propagated: cells around (-72, -72) are valid coordinates.
    CHECK_FALSE(m.is_occupied({-72.0, -72.0}));
}

TEST_CASE("FieldMap: JSON load — explicit origin array") {
    const std::string json = R"({
        "size_in": [50, 50],
        "origin": [0, 0],
        "obstacles": [
            {"type": "rect", "x": 10, "y": 10, "w": 5, "h": 5}
        ]
    })";
    FieldMap m = FieldMap::load_from_json_string(json);
    CHECK(m.config().origin_world_xy.x == doctest::Approx(0.0));
    CHECK(m.config().origin_world_xy.y == doctest::Approx(0.0));
    CHECK(m.is_occupied({12.5, 12.5}));
}

TEST_CASE("FieldMap: JSON load — malformed input throws clear error") {
    const std::string bad = R"({ "size_in": [144, 144], "obstacles": [ {"type": "rect" )";
    CHECK_THROWS_AS(FieldMap::load_from_json_string(bad), std::runtime_error);

    const std::string bad_type = R"({
        "size_in": [144, 144], "origin": "center",
        "obstacles": [{"type": "trapezoid", "x": 0, "y": 0}]
    })";
    CHECK_THROWS_AS(FieldMap::load_from_json_string(bad_type), std::runtime_error);
}

TEST_CASE("FieldMap: JSON load — comments are tolerated") {
    const std::string json = R"({
        // top of file
        "size_in": [144, 144],
        "origin": "center",
        "obstacles": [
            /* a wall */
            {"type": "rect", "x": 0, "y": 0, "w": 5, "h": 5}
        ]
    })";
    FieldMap m = FieldMap::load_from_json_string(json);
    CHECK(m.is_occupied({2.5, 2.5}));
}

TEST_CASE("FieldMap: ray-cast handles rays parallel to grid axes without divide-by-zero") {
    FieldMap m{};
    m.add_rect_obstacle(-10.0, 30.0, 20.0, 1.0);
    // +Y ray.
    const double d_pos = m.cast_ray({0.0, 0.0}, k_pi / 2.0, k_max_range);
    CHECK(d_pos > 29.0);
    CHECK(d_pos < 32.0);

    // -Y ray with no obstacle in that direction.
    const double d_neg = m.cast_ray({0.0, 0.0}, -k_pi / 2.0, 50.0);
    CHECK(d_neg == doctest::Approx(51.0));
}

TEST_CASE("FieldMap: invalid resolution / size throws") {
    FieldMap::Config bad_res{};
    bad_res.resolution_in = 0.0;
    CHECK_THROWS_AS(FieldMap{bad_res}, std::runtime_error);

    FieldMap::Config bad_size{};
    bad_size.size_x_in = 0.0;
    CHECK_THROWS_AS(FieldMap{bad_size}, std::runtime_error);
}

TEST_CASE("FieldMap: cells_x / cells_y match config") {
    FieldMap m{};
    CHECK(m.cells_x() == 144);
    CHECK(m.cells_y() == 144);

    FieldMap::Config c{};
    c.size_x_in     = 100.0;
    c.size_y_in     = 50.0;
    c.resolution_in = 0.5;
    FieldMap m2{c};
    CHECK(m2.cells_x() == 200);
    CHECK(m2.cells_y() == 100);
}
