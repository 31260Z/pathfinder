#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <memory>
#include <stdexcept>

namespace pathfinder {

// IMU mounting orientation, expressed as (where the IMU's +Z axis points in
// bot frame) × (where the IMU's +X axis points in bot frame). 8 cardinal
// mountings — every combination of "Z up vs down" and "X forward / backward
// / left / right". The spec §6 mentions "6 cardinal mountings"; that count
// is imprecise. The full set of orientations that keep the IMU's body axes
// aligned with bot axes is 8 (2 Z-choices × 4 in-plane X-choices). Mountings
// where the IMU sits on its side (Z is horizontal in bot-frame) are excluded
// — they require a generic 3D rotation, not an enum, and almost no team
// uses them in practice.
//
// Sign-flip table (yaw_sign · raw_reading = bot-frame reading):
//   ZUp_*    : IMU+Z = bot+Z   → flip yaw sign (RH/CCW about +Z = CCW from above = -CW)
//   ZDown_*  : IMU+Z = -bot+Z  → no flip       (RH/CCW about -Z = CW from above)
// In-plane X-orientation does not affect the *sign* of yaw — turning the IMU
// 180° about its own +Z axis doesn't flip the sign of yaw or yaw rate. It
// would change the heading *zero*, but we expect the user (or
// chassis.calibrate()) to zero the IMU when the bot is facing the desired
// reference heading, which absorbs that constant. The mounting therefore
// only contributes a sign multiplier here.
enum class ImuMounting {
    ZUp_XForward,
    ZUp_XBackward,
    ZUp_XLeft,
    ZUp_XRight,
    ZDown_XForward,
    ZDown_XBackward,
    ZDown_XLeft,
    ZDown_XRight,
};

namespace detail {

// Sign multiplier on yaw and yaw-rate readings: ZDown mountings already
// match Pathfinder's CW+ convention; ZUp mountings need negation.
inline double imu_yaw_sign(ImuMounting m) {
    switch (m) {
        case ImuMounting::ZUp_XForward:
        case ImuMounting::ZUp_XBackward:
        case ImuMounting::ZUp_XLeft:
        case ImuMounting::ZUp_XRight:
            return -1.0;
        case ImuMounting::ZDown_XForward:
        case ImuMounting::ZDown_XBackward:
        case ImuMounting::ZDown_XLeft:
        case ImuMounting::ZDown_XRight:
            return +1.0;
    }
    throw std::invalid_argument("imu_yaw_sign: unknown ImuMounting");
}

} // namespace detail

// User-facing IMU config struct.
struct Imu {
    std::shared_ptr<IImu> sensor;
    ImuMounting           mounting             = ImuMounting::ZUp_XForward;
    Vector2               offset_xy_in         = {0.0, 0.0};
    double                offset_z_in          = 0.0;
    double                gyro_bias_dps        = 0.0;
    double                gyro_sigma_dps_rt_hz = 0.0;

    // Returns the IMU's heading in Pathfinder's bot-frame convention
    // (CW-positive, +X-forward). Applies the mounting yaw-sign flip and
    // normalizes to (-π, π]. The expectation is the IMU was zeroed at the
    // bot's reference heading (`chassis.calibrate()`); the mounting only
    // contributes a sign here.
    Angle heading_in_bot_frame() const {
        const double raw = sensor->heading_rad().rad;
        return Angle{detail::imu_yaw_sign(mounting) * raw}.normalize_signed();
    }

    // Yaw rate in bot frame, sign-flipped per mounting.
    double yaw_rate_in_bot_frame() const {
        return detail::imu_yaw_sign(mounting) * sensor->yaw_rate_rad_s();
    }
};

} // namespace pathfinder
