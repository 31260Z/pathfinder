#pragma once

#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/geometry/named_point.hpp>
#include <pathfinder/geometry/params.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace pathfinder {

class Bot {
  public:
    Bot() = default;

    Bot& footprint(double length, double width) {
        footprint_ = {length, width};
        return *this;
    }

    Bot& origin(Corner corner) {
        origin_ = corner;
        return *this;
    }

    template <typename... Params>
        requires (sizeof...(Params) > 0)
    Bot& point(std::string_view name, Params... params) {
        const Vector2 offset = resolve_point(params...);
        points_.push_back(NamedPoint{std::string(name), offset});
        return *this;
    }

    const NamedPoint& point(std::string_view name) const {
        for (const auto& p : points_) {
            if (p.name == name) return p;
        }
        throw std::out_of_range("Bot::point: no point named '" + std::string(name) + "'");
    }

    Footprint footprint() const { return footprint_; }
    Corner origin() const { return origin_; }
    const std::vector<NamedPoint>& points() const { return points_; }

  private:
    Footprint footprint_{};
    Corner origin_ = Corner::BackLeft;
    std::vector<NamedPoint> points_;
};

} // namespace pathfinder
