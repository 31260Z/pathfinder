#pragma once

// Umbrella include for the PROS adapter layer (Wave G). User PROS projects
// pull this into their `main.cpp` to get everything in one shot:
//
//   #include <pathfinder/pathfinder.hpp>     // core (PROS-free)
//   #include <pathfinder/pros/all.hpp>       // PROS adapters + factories
//
// The PROS adapters all live under namespace `pathfinder::pros_v5` (a `_v5`
// suffix to avoid colliding with any future `pathfinder::pros::*` runtime
// namespace and to make the source location obvious in error messages).
//
// IMPORTANT: this header transitively includes `pros/...` files. Do NOT
// pull it into a host-side build path. The library's main umbrella
// (`<pathfinder/pathfinder.hpp>`) is intentionally PROS-free.

#include <pathfinder/pros/factories.hpp>
#include <pathfinder/pros/pros_controller.hpp>
#include <pathfinder/pros/pros_distance.hpp>
#include <pathfinder/pros/pros_gps.hpp>
#include <pathfinder/pros/pros_imu.hpp>
#include <pathfinder/pros/pros_motor.hpp>
#include <pathfinder/pros/pros_rotation.hpp>
