#pragma once

#include <pathfinder/odometry/field_map.hpp>

#include <cstddef>

namespace pathfinder {

// Localization-tier selector. Constructed in one of three ways at chassis
// construction time:
//
//   Chassis(... , Localization::DeadReckoning,                                ...);
//   Chassis(... , Localization::Ekf,                                          ...);
//   Chassis(... , Localization::Mcl{ .field_map = my_map, .particles = 300 }, ...);
//
// The first two are tag types (empty structs with inline-constexpr
// instances); MCL takes parameters so it's a value type and carries the
// field map the particle filter ray-casts against.
namespace Localization {

struct DeadReckoning_t {};
inline constexpr DeadReckoning_t DeadReckoning{};

struct Ekf_t {};
inline constexpr Ekf_t Ekf{};

struct Mcl {
    FieldMap        field_map = FieldMap::default_perimeter();
    std::size_t     particles = 300;
};

} // namespace Localization
} // namespace pathfinder
