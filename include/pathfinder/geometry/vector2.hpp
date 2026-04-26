#pragma once

#include <cmath>
#include <stdexcept>

namespace pathfinder {

inline constexpr double k_vector2_epsilon = 1e-9;

struct Vector2 {
    double x = 0.0;
    double y = 0.0;

    constexpr Vector2() = default;
    constexpr Vector2(double x_in, double y_in) : x(x_in), y(y_in) {}

    constexpr Vector2 operator+(Vector2 rhs) const { return {x + rhs.x, y + rhs.y}; }
    constexpr Vector2 operator-(Vector2 rhs) const { return {x - rhs.x, y - rhs.y}; }
    constexpr Vector2 operator-() const { return {-x, -y}; }
    constexpr Vector2 operator*(double s) const { return {x * s, y * s}; }
    constexpr Vector2 operator/(double s) const { return {x / s, y / s}; }

    constexpr Vector2& operator+=(Vector2 rhs) { x += rhs.x; y += rhs.y; return *this; }
    constexpr Vector2& operator-=(Vector2 rhs) { x -= rhs.x; y -= rhs.y; return *this; }
    constexpr Vector2& operator*=(double s) { x *= s; y *= s; return *this; }
    constexpr Vector2& operator/=(double s) { x /= s; y /= s; return *this; }

    bool operator==(Vector2 rhs) const {
        return std::abs(x - rhs.x) <= k_vector2_epsilon
            && std::abs(y - rhs.y) <= k_vector2_epsilon;
    }
    bool operator!=(Vector2 rhs) const { return !(*this == rhs); }
};

constexpr Vector2 operator*(double s, Vector2 v) { return {v.x * s, v.y * s}; }

constexpr double dot(Vector2 a, Vector2 b) { return a.x * b.x + a.y * b.y; }

constexpr double cross(Vector2 a, Vector2 b) { return a.x * b.y - a.y * b.x; }

// 90-degree CCW rotation. Used by the wheel-finder math (spec Appendix A).
constexpr Vector2 perp(Vector2 v) { return {-v.y, v.x}; }

constexpr double norm_sq(Vector2 v) { return v.x * v.x + v.y * v.y; }

inline double norm(Vector2 v) { return std::sqrt(norm_sq(v)); }

// Returns the unit vector. Throws std::domain_error on zero-length input.
inline Vector2 normalize(Vector2 v) {
    const double n = norm(v);
    if (n <= k_vector2_epsilon) {
        throw std::domain_error("normalize: zero-length vector");
    }
    return {v.x / n, v.y / n};
}

inline Vector2 rotate(Vector2 v, double angle_rad) {
    const double c = std::cos(angle_rad);
    const double s = std::sin(angle_rad);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}

inline double distance(Vector2 a, Vector2 b) { return norm(b - a); }

// Signed angle from `from` to `to` in radians, in (-pi, pi].
inline double angle_to(Vector2 from, Vector2 to) {
    return std::atan2(cross(from, to), dot(from, to));
}

struct Vector3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    constexpr Vector3() = default;
    constexpr Vector3(double x_in, double y_in, double z_in) : x(x_in), y(y_in), z(z_in) {}

    constexpr Vector3 operator+(Vector3 rhs) const { return {x + rhs.x, y + rhs.y, z + rhs.z}; }
    constexpr Vector3 operator-(Vector3 rhs) const { return {x - rhs.x, y - rhs.y, z - rhs.z}; }
    constexpr Vector3 operator-() const { return {-x, -y, -z}; }
    constexpr Vector3 operator*(double s) const { return {x * s, y * s, z * s}; }
    constexpr Vector3 operator/(double s) const { return {x / s, y / s, z / s}; }

    constexpr Vector3& operator+=(Vector3 rhs) { x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
    constexpr Vector3& operator-=(Vector3 rhs) { x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
    constexpr Vector3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
    constexpr Vector3& operator/=(double s) { x /= s; y /= s; z /= s; return *this; }

    bool operator==(Vector3 rhs) const {
        return std::abs(x - rhs.x) <= k_vector2_epsilon
            && std::abs(y - rhs.y) <= k_vector2_epsilon
            && std::abs(z - rhs.z) <= k_vector2_epsilon;
    }
    bool operator!=(Vector3 rhs) const { return !(*this == rhs); }
};

constexpr Vector3 operator*(double s, Vector3 v) { return {v.x * s, v.y * s, v.z * s}; }

} // namespace pathfinder
