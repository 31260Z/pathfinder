#pragma once

// Top-level umbrella header for Pathfinder. User `main.cpp` files include
// this once to pull in everything they're likely to touch (per spec §13/§14).
// Power users may include sub-module umbrellas (e.g. <pathfinder/sensors/
// sensors.hpp>) directly for slightly better build times.

#include <pathfinder/autonomous/autonomous.hpp>
#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/controllers/controllers.hpp>
#include <pathfinder/driving/driving.hpp>
#include <pathfinder/geometry/geometry.hpp>
#include <pathfinder/odometry/odometry.hpp>
#include <pathfinder/sensors/sensors.hpp>
