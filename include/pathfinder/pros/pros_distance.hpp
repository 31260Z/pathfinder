#pragma once

// PROS-side concrete IDistance adapter. Wraps `pros::Distance`.
//   distance_in() → distance_.get_distance() millimeters → inches (×0.0393701)
//   is_valid()    → PROS reports raw distance ≥ 9999 (or PROS_ERR == -1) for
//                   "no detection". Treat anything below ~9990mm as valid.
//
// Per the V5 Distance Sensor docs, the maximum useful range is ~2 m; readings
// near or beyond that are returned as 9999 mm and effectively mean "no
// object." We surface that to consumers via `is_valid()` so the EKF / MCL
// can drop the observation cleanly.

#include <pathfinder/sensors/interfaces.hpp>

#include <pros/distance.hpp>

#include <cstdint>

namespace pathfinder::pros_v5 {

class PROSDistance : public IDistance {
  public:
    explicit PROSDistance(int port)
        : dist_(static_cast<std::uint8_t>(port)) {}

    double distance_in() const override {
        const std::int32_t mm = dist_.get_distance();
        if (mm < 0) return 0.0;     // PROS_ERR — caller should also check is_valid()
        // 1 mm = 0.0393701 inches. Use the standard conversion factor.
        return static_cast<double>(mm) * 0.0393700787401575;
    }

    bool is_valid() const override {
        const std::int32_t mm = dist_.get_distance();
        // PROS returns -1 for an error and ~9999 for "no object" near the
        // max range. Anything below 9990 is a real measurement.
        return (mm >= 0) && (mm < 9990);
    }

  private:
    mutable pros::Distance dist_;
};

} // namespace pathfinder::pros_v5
