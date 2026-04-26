#pragma once

#include <cstddef>

namespace pathfinder {

// Localization-tier selector. Constructed in one of three ways at chassis
// construction time:
//
//   Chassis(... , Localization::DeadReckoning,           ...);
//   Chassis(... , Localization::Ekf,                     ...);
//   Chassis(... , Localization::Mcl{ .particles = 300 }, ...);
//
// The first two are tag types (empty structs with inline-constexpr
// instances); MCL takes parameters so it's a value type.
//
// Wave C wires up DR only — the Ekf and Mcl tags are accepted but currently
// fall back to DR with a warning (see `Chassis::warn_unimplemented_tier`).
// Wave F replaces the fallback with real estimators.
namespace Localization {

struct DeadReckoning_t {};
inline constexpr DeadReckoning_t DeadReckoning{};

struct Ekf_t {};
inline constexpr Ekf_t Ekf{};

struct Mcl {
    // FieldMap will be added in Wave F; placeholder.
    std::size_t particles = 300;
};

} // namespace Localization
} // namespace pathfinder
