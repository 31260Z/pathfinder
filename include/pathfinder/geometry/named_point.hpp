#pragma once

#include <pathfinder/geometry/vector2.hpp>

#include <string>

namespace pathfinder {

struct NamedPoint {
    std::string name;
    Vector2 offset;
};

} // namespace pathfinder
