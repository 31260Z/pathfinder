#pragma once

#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

namespace pathfinder {

class Sensors;   // forward decl — interface header doesn't need the full type

// Polymorphic localization interface. The chassis holds one of these via
// std::unique_ptr; a concrete adapter wraps DeadReckoning, Ekf, or Mcl and
// presents a uniform predict/observe/getter surface to the chassis.
class ILocalizer {
public:
    virtual ~ILocalizer() = default;

    // Predict step. Caller passes the body-frame wheel-displacement deltas
    // (subtracting the rotation contribution is the chassis's job — these
    // are bot-frame translations of the WHEEL CENTER over the period dt_sec)
    // plus the absolute heading from the (fused) IMU and the period itself.
    // Each adapter does what makes sense for its underlying estimator.
    virtual void predict(double parallel_wheel_delta_in,
                          double perp_wheel_delta_in,
                          Angle  heading_now,
                          double dt_sec) = 0;

    // Process additional measurement-style sensor observations. DR ignores
    // these; EKF runs measurement updates for landmarks (VexGps observations
    // arrive as ILandmark observations too); MCL runs distance + landmark
    // updates against its field map.
    virtual void process_observations(const Sensors& sensors) = 0;

    virtual Pose2        pose() const = 0;
    virtual BodyVelocity body_velocity() const = 0;
    virtual Matrix3      pose_covariance() const = 0;
    virtual void         set_pose(Pose2 new_pose, bool keep_velocity = false) = 0;
    virtual void         reset() = 0;
};

} // namespace pathfinder
