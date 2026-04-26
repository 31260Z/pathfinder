#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/sensors/interfaces.hpp>
#include <pathfinder/sensors/wheel_specs.hpp>

#include <memory>

namespace pathfinder {

// Whether a tracking wheel rolls along bot-X (parallel, "forward") or bot-Y
// (perpendicular, "lateral"). Spec §6 — the odometry math projects encoder
// revolutions onto the wheel's rolling axis, so the consumer needs to know
// which axis it is.
enum class Axis { X, Y };

// User-facing config for one tracking wheel pod. The user constructs this
// struct (typically in the `Sensors()` builder) describing where the wheel
// sits, what direction it rolls, and how noisy its encoder is.
//
// `offset` is in the user's chosen-corner bot frame (e.g. back-left at
// (0, 0)). The chassis layer translates these to center-relative offsets
// before handing them to `DeadReckoning::Config` — see
// `corner_to_center_offset()` in `frame_helpers.hpp`.
struct TrackingWheel {
    std::shared_ptr<IRotation> sensor;
    Wheel                      wheel              = Wheel::Omni_275;
    double                     custom_diameter_in = 0.0;   // used iff wheel == Custom
    Axis                       axis               = Axis::X;
    Vector2                    offset             = {0.0, 0.0};
    double                     gear_ratio         = 1.0;   // wheel revs per encoder rev (rare; usually 1)
    double                     sigma_along_in     = 0.02;  // along-roll noise std-dev
    double                     sigma_lateral_in   = 0.10;  // perpendicular slip noise std-dev

    // Returns the wheel diameter in inches. Resolves `Wheel::Custom` to the
    // explicit `custom_diameter_in` field rather than throwing.
    double diameter_in() const {
        return (wheel == Wheel::Custom) ? custom_diameter_in : wheel_diameter_in(wheel);
    }

    // Convert encoder revolutions (already accounting for sensor reversal at
    // the `IRotation` layer) into inches of wheel travel along its rolling
    // axis. `wheel_revs = encoder_revs · gear_ratio`, then ×π·d.
    double encoder_to_inches(double encoder_revs) const {
        return encoder_revs * gear_ratio * k_pi * diameter_in();
    }
};

} // namespace pathfinder
