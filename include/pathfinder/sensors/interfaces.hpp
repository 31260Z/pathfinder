#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>

#include <optional>

// Sensor interfaces. Pure virtual; concrete implementations live in the PROS
// adapter layer (Wave G) or, for tests, in `mocks.hpp`. Pathfinder's odometry,
// EKF, MCL, and chassis layers depend only on these interfaces — they never
// touch a `pros::Rotation` or `pros::Imu` directly. This is the seam through
// which the library stays host-testable (CLAUDE.md "PROS isolation").
//
// Unit conventions at the interface boundary (see Wave B brief §"Architecture"):
//   IRotation::position_revolutions()  — wheel-shaft revolutions (PROS centidegrees → revs at the wrapper)
//   IRotation::velocity_rps()          — revolutions per second
//   IImu::heading_rad()                — absolute heading in (-π, π]
//   IImu::yaw_rate_rad_s()             — rad/s
//   IDistance::distance_in()           — inches; check `is_valid()` for "no object"
//   ILandmark::poll()                  — optional observation, when one is fresh

namespace pathfinder {

class IRotation {
  public:
    virtual ~IRotation() = default;
    virtual double position_revolutions() const = 0;
    virtual double velocity_rps() const = 0;
    virtual void   reset_position(double new_revs = 0.0) = 0;
    virtual bool   is_reversed() const = 0;
};

class IImu {
  public:
    virtual ~IImu() = default;
    virtual Angle  heading_rad() const = 0;
    virtual double yaw_rate_rad_s() const = 0;
    virtual bool   is_calibrating() const = 0;
    virtual void   reset_heading(Angle new_heading = Angle{}) = 0;
};

class IDistance {
  public:
    virtual ~IDistance() = default;
    virtual double distance_in() const = 0;
    virtual bool   is_valid() const = 0;
};

// A landmark observation: the bot saw a fixture (AprilTag, GPS-style beacon,
// VEX GPS origin, ...) at a known field pose, and observed it at a relative
// pose in the bot's frame. `covariance` is the 3×3 noise model for the
// (range, bearing, [heading]) tuple, consumed by the EKF / MCL.
struct LandmarkObservation {
    Pose2   landmark_field_pose;
    Pose2   relative_observation;
    Matrix3 covariance{};
};

class ILandmark {
  public:
    virtual ~ILandmark() = default;

    // Returns std::nullopt when no fresh observation is available (e.g. the
    // tag wasn't in the camera frame this tick). The estimator only consumes
    // populated observations.
    virtual std::optional<LandmarkObservation> poll() = 0;
};

} // namespace pathfinder
