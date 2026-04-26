#pragma once

// PROS-side concrete IRotation adapter. Wraps `pros::Rotation` and
// unit-converts per Pathfinder's interface contract:
//   position_revolutions() → rot_.get_position() centidegrees → revs (÷36000)
//   velocity_rps()         → rot_.get_velocity() centidegrees/s → revs/s
//   reset_position(revs)   → rot_.set_position(int centidegrees)
//   is_reversed()          → returns the constructor flag
//
// PROS isolation: includes <pros/rotation.hpp>; not compiled on host. The
// reversal flag is also pushed into the PROS device via `set_reversed`.

#include <pathfinder/driving/interfaces.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <pros/rotation.hpp>

#include <cmath>
#include <cstdint>

namespace pathfinder::pros_v5 {

class PROSRotation : public IRotation {
  public:
    explicit PROSRotation(int port, bool reversed = false)
        : rot_(static_cast<std::int8_t>(port)), reversed_(reversed) {
        rot_.set_reversed(reversed_);
    }

    double position_revolutions() const override {
        // PROS get_position returns centidegrees (1/100 of a degree).
        // 360° = 36000 centidegrees per revolution.
        const std::int32_t cdeg = rot_.get_position();
        const double revs = static_cast<double>(cdeg) / 36000.0;
        // The PROS device already accounts for `set_reversed` at the read
        // layer. Per the IRotation contract, sensor adapters report
        // already-signed values — so we DON'T double-flip here.
        return revs;
    }

    double velocity_rps() const override {
        const std::int32_t cdeg_s = rot_.get_velocity();
        return static_cast<double>(cdeg_s) / 36000.0;
    }

    void reset_position(double new_revs = 0.0) override {
        // PROS set_position takes centidegrees.
        const auto cdeg = static_cast<std::int32_t>(std::lround(new_revs * 36000.0));
        rot_.set_position(cdeg);
    }

    bool is_reversed() const override { return reversed_; }

  private:
    pros::Rotation rot_;
    bool           reversed_;
};

} // namespace pathfinder::pros_v5
