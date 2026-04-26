#pragma once

// PROS-side concrete ILandmark adapter for the VEX GPS sensor. The GPS
// provides an absolute pose (x, y, yaw); we wrap that into a
// LandmarkObservation whose `landmark_field_pose` is the world origin
// (0, 0, 0) and whose `relative_observation` is the bot's pose itself.
// The EKF / MCL fuses it as a direct pose measurement at the bot center.
//
// Unit conversions:
//   PROS gps_status_s_t.x, .y are in METERS — multiply by 39.3700787 to
//                                              get inches.
//   PROS gps_status_s_t.yaw is in DEGREES (PROS' compass-style frame; we
//                                          re-normalize to (-π, π]).
//
// Covariance defaults: the V5 GPS spec gives ~1" position accuracy + ~1°
// heading accuracy under good lighting; conservatively we use σ_xy = 1.0",
// σ_θ = 5° here. Users can subclass + override `poll()` if they want to
// tune these per-bot.

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <pros/gps.hpp>

#include <cmath>
#include <cstdint>
#include <optional>

namespace pathfinder::pros_v5 {

class PROSGps : public ILandmark {
  public:
    explicit PROSGps(int port) : gps_(static_cast<std::uint8_t>(port)) {}

    std::optional<LandmarkObservation> poll() override {
        // Drop the observation when the GPS reports any error. PROS' GPS
        // get_error returns the L2 reprojection error in meters; PROS
        // documentation says values > 0 are still "OK but uncertain", but
        // a non-zero error >> 0.05 m typically means the sensor lost its
        // field-tile lock. Use a conservative threshold (0.50 m).
        const double err_m = gps_.get_error();
        if (err_m > 0.50) return std::nullopt;

        const auto status = gps_.get_position_and_orientation();

        constexpr double k_m_to_in = 39.3700787401575;
        Pose2 measured;
        measured.x       = status.x * k_m_to_in;
        measured.y       = status.y * k_m_to_in;
        measured.heading = Angle::degrees(status.yaw).normalize_signed();

        LandmarkObservation obs{};
        obs.landmark_field_pose  = Pose2{};         // world origin
        obs.relative_observation = measured;        // measured pose IS the relative obs
        // diag(σ_xy², σ_xy², σ_θ²); σ_xy = 1.0 in, σ_θ = 5° → ≈ 0.0762 rad
        constexpr double sigma_xy_in = 1.0;
        constexpr double sigma_th_rad = 5.0 * k_deg_to_rad;
        obs.covariance.m[0][0] = sigma_xy_in * sigma_xy_in;
        obs.covariance.m[1][1] = sigma_xy_in * sigma_xy_in;
        obs.covariance.m[2][2] = sigma_th_rad * sigma_th_rad;
        return obs;
    }

  private:
    mutable pros::Gps gps_;
};

} // namespace pathfinder::pros_v5
