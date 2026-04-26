#pragma once

// PROS-side controller adapter — produces a `std::function<double(JoystickAxis)>`
// that the chassis's opcontrol modes consume. Each axis is normalized to
// [-1, 1] (PROS get_analog returns ±127 raw, divided by 127 here).
//
// Usage:
//   pros::Controller ctl(pros::E_CONTROLLER_MASTER);
//   auto poll = make_axis_reader(ctl);
//   chassis.opcontrol(FlightStyle::Rate{}, poll,
//                     []{ return pros::competition::is_disabled(); });

#include <pathfinder/chassis/opcontrol_modes.hpp>

#include <pros/misc.hpp>

#include <functional>

namespace pathfinder::pros_v5 {

inline std::function<double(JoystickAxis)>
make_axis_reader(pros::Controller& ctl) {
    return [&ctl](JoystickAxis axis) -> double {
        pros::controller_analog_e_t pros_axis = pros::E_CONTROLLER_ANALOG_LEFT_Y;
        switch (axis) {
            case JoystickAxis::LeftX:  pros_axis = pros::E_CONTROLLER_ANALOG_LEFT_X;  break;
            case JoystickAxis::LeftY:  pros_axis = pros::E_CONTROLLER_ANALOG_LEFT_Y;  break;
            case JoystickAxis::RightX: pros_axis = pros::E_CONTROLLER_ANALOG_RIGHT_X; break;
            case JoystickAxis::RightY: pros_axis = pros::E_CONTROLLER_ANALOG_RIGHT_Y; break;
        }
        const std::int32_t raw = ctl.get_analog(pros_axis);   // ±127
        return static_cast<double>(raw) / 127.0;
    };
}

} // namespace pathfinder::pros_v5
