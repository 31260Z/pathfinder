#pragma once

// PROS-side concrete IImu adapter. Wraps `pros::Imu`. Unit + frame
// conventions:
//   heading_rad()   → imu_.get_heading() degrees [0,360) → radians, normalized to (-π, π]
//   yaw_rate_rad_s()→ imu_.get_gyro_rate().z degrees/s → radians/s
//   reset_heading(h)→ imu_.set_heading(h.degrees())  (PROS expects [0, 360))
//   is_calibrating()→ imu_.is_calibrating()
//
// The bot-frame sign-flip (RH/CCW+ yaw → Pathfinder's CW+ heading) is
// applied at the `Imu` config struct level (see sensors/imu.hpp's
// `heading_in_bot_frame()` / `yaw_rate_in_bot_frame()` and the per-mounting
// sign table). This adapter exposes the IMU's raw reading; the higher
// layer handles the mounting transform. That keeps the adapter trivial and
// the mounting metadata in one place.

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <pros/imu.hpp>

#include <cstdint>

namespace pathfinder::pros_v5 {

class PROSImu : public IImu {
  public:
    explicit PROSImu(int port) : imu_(static_cast<std::uint8_t>(port)) {}

    Angle heading_rad() const override {
        // PROS get_heading returns degrees in [0, 360). Convert to radians,
        // then normalize to (-π, π] so downstream code uses one canonical
        // form (matches IImu contract).
        const double deg = imu_.get_heading();
        return Angle::degrees(deg).normalize_signed();
    }

    double yaw_rate_rad_s() const override {
        // gyro.z is in degrees/sec around the IMU's body Z axis.
        const auto gyro = imu_.get_gyro_rate();
        return gyro.z * k_deg_to_rad;
    }

    bool is_calibrating() const override { return imu_.is_calibrating(); }

    void reset_heading(Angle new_heading = Angle{}) override {
        // PROS set_heading expects degrees in [0, 360). Normalize first.
        const double deg_norm = new_heading.normalize_unsigned().degrees();
        imu_.set_heading(deg_norm);
    }

  private:
    pros::Imu imu_;
};

} // namespace pathfinder::pros_v5
