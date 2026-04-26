#pragma once

#include <pathfinder/geometry/vector2.hpp>

#include <array>

namespace pathfinder {

enum class Corner { FrontLeft, FrontRight, BackLeft, BackRight };

struct Footprint {
    double length = 0.0;
    double width = 0.0;

    // Returns FrontLeft, FrontRight, BackLeft, BackRight in that order, in
    // bot-frame coordinates given which corner is the origin (0, 0).
    // Bot frame: +X = forward, +Y = right.
    std::array<Vector2, 4> corners(Corner origin) const {
        double x_back = 0.0, x_front = 0.0, y_left = 0.0, y_right = 0.0;
        switch (origin) {
            case Corner::BackLeft:
                x_back = 0.0;       x_front = length;
                y_left = 0.0;       y_right = width;
                break;
            case Corner::BackRight:
                x_back = 0.0;       x_front = length;
                y_left = -width;    y_right = 0.0;
                break;
            case Corner::FrontLeft:
                x_back = -length;   x_front = 0.0;
                y_left = 0.0;       y_right = width;
                break;
            case Corner::FrontRight:
                x_back = -length;   x_front = 0.0;
                y_left = -width;    y_right = 0.0;
                break;
        }
        return {{
            {x_front, y_left},   // FrontLeft
            {x_front, y_right},  // FrontRight
            {x_back, y_left},    // BackLeft
            {x_back, y_right},   // BackRight
        }};
    }

    Footprint inflate(double margin) const {
        return {length + 2.0 * margin, width + 2.0 * margin};
    }

    bool contains(Vector2 point, Corner origin) const {
        const auto cs = corners(origin);
        const double x_min = cs[2].x;  // BackLeft.x == back x
        const double x_max = cs[0].x;  // FrontLeft.x == front x
        const double y_min = cs[0].y;  // FrontLeft.y == left y
        const double y_max = cs[1].y;  // FrontRight.y == right y
        return point.x >= x_min && point.x <= x_max
            && point.y >= y_min && point.y <= y_max;
    }
};

} // namespace pathfinder
