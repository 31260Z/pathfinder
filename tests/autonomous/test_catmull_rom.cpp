#include <doctest/doctest.h>

#include <pathfinder/autonomous/catmull_rom.hpp>

#include <cmath>
#include <vector>

using pathfinder::Vector2;
using pathfinder::distance;
using pathfinder::norm;
using pathfinder::catmull_rom::Path;

TEST_CASE("Catmull-Rom: path passes through every waypoint") {
    std::vector<Vector2> wpts{
        {0, 0}, {12, 6}, {24, -3}, {36, 0}, {48, 12},
    };
    Path path(wpts);
    for (std::size_t i = 0; i < wpts.size(); ++i) {
        const double s = path.waypoint_arclength(i);
        Vector2 p = path.sample_at_arclength(s);
        CHECK(distance(p, wpts[i]) < 0.05);
    }
}

TEST_CASE("Catmull-Rom: total length close to sum of chord lengths for near-straight path") {
    std::vector<Vector2> wpts{
        {0, 0}, {10, 0.1}, {20, -0.05}, {30, 0.02}, {40, 0.0},
    };
    Path path(wpts);
    double chord_sum = 0.0;
    for (std::size_t i = 1; i < wpts.size(); ++i) {
        chord_sum += distance(wpts[i - 1], wpts[i]);
    }
    CHECK(path.total_length_in() == doctest::Approx(chord_sum).epsilon(0.02));
}

TEST_CASE("Catmull-Rom: tangent at midpoint of straight horizontal path is along +X") {
    std::vector<Vector2> wpts{
        {0, 0}, {10, 0}, {20, 0}, {30, 0},
    };
    Path path(wpts);
    const double mid = path.total_length_in() * 0.5;
    Vector2 t = path.tangent_at_arclength(mid);
    CHECK(t.x == doctest::Approx(1.0).epsilon(0.01));
    CHECK(std::abs(t.y) < 0.05);
}

TEST_CASE("Catmull-Rom: closest_point projects onto the path") {
    std::vector<Vector2> wpts{
        {0, 0}, {10, 0}, {20, 0},
    };
    Path path(wpts);
    double s_out = -1.0;
    Vector2 q{5.0, 3.0};
    Vector2 cp = path.closest_point_arclength(q, s_out);
    CHECK(s_out == doctest::Approx(5.0).epsilon(0.05));
    CHECK(distance(cp, Vector2{5.0, 0.0}) < 0.1);
}

TEST_CASE("Catmull-Rom: sample(0) and sample(1) are endpoints") {
    std::vector<Vector2> wpts{
        {2, 3}, {7, 8}, {12, 5},
    };
    Path path(wpts);
    Vector2 a = path.sample(0.0);
    Vector2 b = path.sample(1.0);
    CHECK(distance(a, wpts.front()) < 0.05);
    CHECK(distance(b, wpts.back())  < 0.05);
}

TEST_CASE("Catmull-Rom: throws on too few waypoints") {
    std::vector<Vector2> single{{0, 0}};
    CHECK_THROWS(Path(single));
}

TEST_CASE("Catmull-Rom: sample_at_arclength matches discretized cumulative length") {
    std::vector<Vector2> wpts{
        {0, 0}, {10, 5}, {20, 0}, {30, -5}, {40, 0},
    };
    Path path(wpts);
    const double total = path.total_length_in();
    Vector2 mid = path.sample_at_arclength(total * 0.5);
    // Just sanity-check that it's somewhere reasonable inside the bounding box.
    CHECK(mid.x > 5.0);
    CHECK(mid.x < 35.0);
}
