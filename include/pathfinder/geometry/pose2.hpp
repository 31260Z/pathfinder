#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <cmath>

namespace pathfinder {

inline constexpr double k_pose_epsilon = 1e-9;

struct Pose2 {
    double x = 0.0;
    double y = 0.0;
    Angle heading{};

    constexpr Pose2() = default;
    constexpr Pose2(double x_in, double y_in, Angle h) : x(x_in), y(y_in), heading(h) {}
    constexpr Pose2(Vector2 t, Angle h) : x(t.x), y(t.y), heading(h) {}

    constexpr Vector2 translation() const { return {x, y}; }

    // compose(child) = world_T_child if `this` is world_T_self and `child` is self_T_child.
    // i.e. applies `this` first, then `child`.
    Pose2 compose(const Pose2& child) const {
        const double c = std::cos(heading.rad);
        const double s = std::sin(heading.rad);
        return Pose2{
            x + c * child.x - s * child.y,
            y + s * child.x + c * child.y,
            heading + child.heading,
        };
    }

    Pose2 inverse() const {
        const double c = std::cos(heading.rad);
        const double s = std::sin(heading.rad);
        return Pose2{
            -(c * x + s * y),
            -(-s * x + c * y),
            Angle{-heading.rad},
        };
    }

    Vector2 transform(Vector2 local_point) const {
        const double c = std::cos(heading.rad);
        const double s = std::sin(heading.rad);
        return {
            x + c * local_point.x - s * local_point.y,
            y + s * local_point.x + c * local_point.y,
        };
    }

    Vector2 inverse_transform(Vector2 world_point) const {
        const double c = std::cos(heading.rad);
        const double s = std::sin(heading.rad);
        const double dx = world_point.x - x;
        const double dy = world_point.y - y;
        return {c * dx + s * dy, -s * dx + c * dy};
    }

    bool operator==(const Pose2& rhs) const {
        return std::abs(x - rhs.x) <= k_pose_epsilon
            && std::abs(y - rhs.y) <= k_pose_epsilon
            && heading == rhs.heading;
    }
    bool operator!=(const Pose2& rhs) const { return !(*this == rhs); }
};

} // namespace pathfinder
