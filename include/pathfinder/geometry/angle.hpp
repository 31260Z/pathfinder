#pragma once

#include <cmath>

namespace pathfinder {

inline constexpr double k_pi = 3.14159265358979323846;
inline constexpr double k_two_pi = 2.0 * k_pi;
inline constexpr double k_deg_to_rad = k_pi / 180.0;
inline constexpr double k_rad_to_deg = 180.0 / k_pi;
inline constexpr double k_angle_epsilon = 1e-9;

struct Angle {
    double rad = 0.0;

    constexpr Angle() = default;
    constexpr explicit Angle(double r) : rad(r) {}

    static constexpr Angle radians(double r) { return Angle{r}; }
    static constexpr Angle degrees(double d) { return Angle{d * k_deg_to_rad}; }

    constexpr double radians() const { return rad; }
    constexpr double degrees() const { return rad * k_rad_to_deg; }

    Angle normalize_signed() const {
        double r = std::fmod(rad + k_pi, k_two_pi);
        if (r <= 0.0) r += k_two_pi;
        return Angle{r - k_pi};
    }

    Angle normalize_unsigned() const {
        double r = std::fmod(rad, k_two_pi);
        if (r < 0.0) r += k_two_pi;
        return Angle{r};
    }

    constexpr Angle operator+(Angle rhs) const { return Angle{rad + rhs.rad}; }
    constexpr Angle operator-(Angle rhs) const { return Angle{rad - rhs.rad}; }
    constexpr Angle operator-() const { return Angle{-rad}; }

    bool operator==(Angle rhs) const { return std::abs(rad - rhs.rad) <= k_angle_epsilon; }
    bool operator!=(Angle rhs) const { return !(*this == rhs); }
};

// Smallest signed delta to rotate from `from` to `to`, in (-pi, pi].
inline Angle shortest_angle(Angle from, Angle to) {
    return (to - from).normalize_signed();
}

} // namespace pathfinder
