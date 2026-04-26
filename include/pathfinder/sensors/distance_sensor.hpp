#pragma once

#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <memory>

namespace pathfinder {

// User-facing config for a single distance sensor (V5 Distance Sensor or
// equivalent). The sensor sits at a known location and orientation on the
// bot; MCL ray-casts against the field map at that ray to compare with the
// reported `distance_in()`.
//
// `theta_deg` follows Pathfinder's bot-frame heading convention: 0° points
// along bot +X (forward); +90° points along bot +Y (right). All offsets are
// in the user's chosen-corner bot frame (translated to center-relative by
// the chassis layer when wiring into MCL).
struct DistanceSensor {
    std::shared_ptr<IDistance> sensor;
    Vector2 offset_xy_in = {0.0, 0.0};
    double  offset_z_in  = 0.0;
    double  theta_deg    = 0.0;     // 0 = +X (forward), +90 = +Y (right)
    double  max_range_in = 79.0;    // V5 distance sensor ceiling
    double  sigma_in     = 0.5;     // measurement noise std-dev
};

} // namespace pathfinder
