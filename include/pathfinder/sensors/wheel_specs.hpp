#pragma once

#include <stdexcept>

namespace pathfinder {

// Standard VEX wheel diameters used for tracking-wheel encoders. The diameter
// converts encoder revolutions to inches of wheel travel via π·d. `Custom`
// defers to `TrackingWheel::custom_diameter_in`.
enum class Wheel {
    Omni_275,      // 2.75" omni
    Omni_325,      // 3.25" omni (VEX "new 4-inch", legacy nomenclature)
    Omni_4,        // 4" omni
    Traction_275,  // 2.75" traction
    Traction_4,    // 4" traction
    Custom,
};

inline double wheel_diameter_in(Wheel w) {
    switch (w) {
        case Wheel::Omni_275:     return 2.75;
        case Wheel::Omni_325:     return 3.25;
        case Wheel::Omni_4:       return 4.0;
        case Wheel::Traction_275: return 2.75;
        case Wheel::Traction_4:   return 4.0;
        case Wheel::Custom:
            // Caller must read TrackingWheel::custom_diameter_in directly;
            // funneling through this helper for `Custom` is a programmer
            // error worth flagging loudly.
            throw std::invalid_argument(
                "wheel_diameter_in: Wheel::Custom — read TrackingWheel::custom_diameter_in instead");
    }
    throw std::invalid_argument("wheel_diameter_in: unknown Wheel value");
}

} // namespace pathfinder
