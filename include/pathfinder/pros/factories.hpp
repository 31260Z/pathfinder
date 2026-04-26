#pragma once

// Convenience port-int-based factories for the PROS adapter classes. Lets
// user code read like the spec §13 cheat sheet:
//
//   auto left  = motor_group({1, 2, 3});
//   auto right = motor_group({4, 5, 6}, /*reversed=*/true);
//   auto i     = imu(7);
//
// All factories return shared_ptrs so they can be stored inside Pathfinder's
// sensor / drivetrain config structs (which use shared_ptr<IFoo> fields).

#include <pathfinder/driving/motor_group.hpp>
#include <pathfinder/pros/pros_distance.hpp>
#include <pathfinder/pros/pros_gps.hpp>
#include <pathfinder/pros/pros_imu.hpp>
#include <pathfinder/pros/pros_motor.hpp>
#include <pathfinder/pros/pros_rotation.hpp>

#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

namespace pathfinder::pros_v5 {

inline std::shared_ptr<IMotor> motor(int port, bool reversed = false) {
    return std::make_shared<PROSMotor>(port, reversed);
}

inline std::shared_ptr<IRotation> rotation_sensor(int port, bool reversed = false) {
    return std::make_shared<PROSRotation>(port, reversed);
}

inline std::shared_ptr<IImu> imu(int port) {
    return std::make_shared<PROSImu>(port);
}

inline std::shared_ptr<IDistance> distance_sensor(int port) {
    return std::make_shared<PROSDistance>(port);
}

inline std::shared_ptr<ILandmark> gps(int port) {
    return std::make_shared<PROSGps>(port);
}

// Build a MotorGroup from a port list. All motors share the same
// `reversed` flag — convenient for an entire drive side that's been
// flipped relative to the cartridge orientation (the most common case).
inline MotorGroup motor_group(std::initializer_list<int> ports, bool reversed = false) {
    std::vector<std::shared_ptr<IMotor>> motors;
    motors.reserve(ports.size());
    for (int p : ports) motors.push_back(motor(p, reversed));
    return MotorGroup{std::move(motors)};
}

// Build a MotorGroup with per-motor reversal flags. Useful for builds with
// adjacent motors mounted upside-down on the same side.
inline MotorGroup motor_group_mixed(
    std::initializer_list<std::pair<int, bool>> port_reversal_pairs) {
    std::vector<std::shared_ptr<IMotor>> motors;
    motors.reserve(port_reversal_pairs.size());
    for (auto [p, rev] : port_reversal_pairs) motors.push_back(motor(p, rev));
    return MotorGroup{std::move(motors)};
}

} // namespace pathfinder::pros_v5
