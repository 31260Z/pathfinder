#pragma once

#include <pathfinder/geometry/bot.hpp>
#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/geometry/vector2.hpp>

namespace pathfinder {

// Returns the bot's geometric center expressed in the user's chosen-corner
// bot frame. e.g. for an 18×18 bot with `Corner::BackLeft` origin, the
// center sits at (length/2, width/2) = (9, 9). For `Corner::FrontRight`, the
// center sits at (-length/2, -width/2) = (-9, -9). Sign convention: the
// chosen corner is at (0, 0); the opposite corner is at (±length, ±width).
inline Vector2 center_in_corner_frame(const Bot& bot) {
    const Footprint fp = bot.footprint();
    const double half_l = fp.length * 0.5;
    const double half_w = fp.width  * 0.5;
    switch (bot.origin()) {
        case Corner::BackLeft:   return { half_l,  half_w};
        case Corner::BackRight:  return { half_l, -half_w};
        case Corner::FrontLeft:  return {-half_l,  half_w};
        case Corner::FrontRight: return {-half_l, -half_w};
    }
    return {0.0, 0.0};   // unreachable; switch is exhaustive
}

// Translate a sensor offset from the user's chosen-corner bot frame to a
// center-relative offset (origin at the bot's geometric center). Odometry-DR
// and the EKF expect center-relative coordinates because their math is
// derived from rotations *about the bot center*.
//
// Math: subtract the center's coordinates in the corner frame from the
// corner-frame offset. Axes are aligned (+X forward, +Y right) regardless of
// which corner is the origin, so this is a pure translation.
inline Vector2 corner_to_center_offset(Vector2 corner_frame_offset, const Bot& bot) {
    return corner_frame_offset - center_in_corner_frame(bot);
}

} // namespace pathfinder
