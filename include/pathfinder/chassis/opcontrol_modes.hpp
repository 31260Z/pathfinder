#pragma once

namespace pathfinder {

// Joystick axis identifiers. The chassis's opcontrol implementation reads
// the axis values via a user-supplied callback, so the enum simply names
// which axis the user wants for each role of each mode.
enum class JoystickAxis { LeftX, LeftY, RightX, RightY };

// Tank-drive: each side's stick controls that side directly. Defaults match
// the spec quick-start.
struct TankDrive {
    JoystickAxis left_axis  = JoystickAxis::LeftY;
    JoystickAxis right_axis = JoystickAxis::RightY;
    double       expo_curve = 0.0;     // 0 = linear; ∈ (0, 1] = exponential weighting
    double       deadband   = 0.05;    // stick fraction below which to clamp to 0
};

// Arcade-drive: one stick is throttle, the other is turn.
struct ArcadeDrive {
    JoystickAxis forward_axis = JoystickAxis::LeftY;
    JoystickAxis turn_axis    = JoystickAxis::RightX;
    double       expo_curve   = 0.0;
    double       deadband     = 0.05;
};

// Curvature-drive ("cheesy drive"): turn input scales with throttle; smooth
// at low speed.
struct CurvatureDrive {
    JoystickAxis throttle_axis  = JoystickAxis::LeftY;
    JoystickAxis curvature_axis = JoystickAxis::RightX;
    double       expo_curve     = 0.0;
    double       deadband       = 0.05;
};

// Flight-style opcontrol modes (spec §10). Stick deflection commands a
// target body rate (linear in/s + yaw deg/s); the chassis closes the loop
// internally so response is consistent across battery voltage and load.
//
// `Rate` is the only flight mode in v1 — `HeadingHold` and `StationKeep`
// were cut from scope (CLAUDE.md "Driver-control modes").
namespace FlightStyle {

struct Rate {
    JoystickAxis forward_axis    = JoystickAxis::LeftY;
    JoystickAxis yaw_axis        = JoystickAxis::RightX;
    double       max_forward_ips = 60.0;     // in/s at full deflection
    double       max_yaw_dps     = 270.0;    // deg/s at full deflection
    double       expo_forward    = 0.4;
    double       expo_yaw        = 0.3;
    double       deadband        = 0.05;
};

} // namespace FlightStyle

} // namespace pathfinder
