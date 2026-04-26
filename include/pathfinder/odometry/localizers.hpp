#pragma once

#include <pathfinder/odometry/dead_reckoning.hpp>
#include <pathfinder/odometry/ekf.hpp>
#include <pathfinder/odometry/field_map.hpp>
#include <pathfinder/odometry/i_localizer.hpp>
#include <pathfinder/odometry/mcl.hpp>
#include <pathfinder/sensors/builder.hpp>
#include <pathfinder/sensors/sensors.hpp>

#include <cmath>

namespace pathfinder {

// ── DeadReckoning adapter ───────────────────────────────────────────────
// Tier 1: pure dead-reckoning. process_observations() is a no-op since DR
// doesn't fuse landmarks; pose_covariance() returns zero (DR has no notion
// of pose uncertainty).
class DrLocalizer : public ILocalizer {
public:
    explicit DrLocalizer(DeadReckoning::Config cfg, Pose2 initial = {})
        : dr_(cfg, initial) {}

    void predict(double par, double perp, Angle heading_now, double dt) override {
        dr_.update(par, perp, heading_now, dt);
    }
    void process_observations(const Sensors& /*sensors*/) override {}

    Pose2        pose() const override            { return dr_.pose(); }
    BodyVelocity body_velocity() const override   { return dr_.body_velocity(); }
    Matrix3      pose_covariance() const override { return Matrix3::zero(); }

    void set_pose(Pose2 p, bool keep_velocity) override { dr_.set_pose(p, keep_velocity); }
    void reset() override                                 { dr_.reset(); }

private:
    DeadReckoning dr_;
};

// ── EKF adapter ─────────────────────────────────────────────────────────
// Tier 2: 6-DOF Gaussian estimator. predict() drives the state with the
// same motion model as DR; process_observations() ingests landmark/GPS
// observations (each LandmarkSensorConfig in the bundle is polled for an
// Observation; on success, EKF::update_landmark applies it). IMU heading
// is already used in predict (the chassis pre-fuses multi-IMU readings),
// so we don't double-count via update_imu_heading here.
class EkfLocalizer : public ILocalizer {
public:
    explicit EkfLocalizer(Ekf::Config cfg, Pose2 initial = {})
        : ekf_(cfg, initial) {}

    void predict(double par, double perp, Angle heading_now, double dt) override {
        ekf_.predict(par, perp, heading_now, dt);
    }

    void process_observations(const Sensors& sensors) override {
        const auto landmarks = sensors.landmark_sensors();
        for (const auto& lm : landmarks) {
            if (!lm.sensor) continue;
            auto obs = lm.sensor->poll();
            if (!obs) continue;
            ekf_.update_landmark(obs->landmark_field_pose,
                                  obs->relative_observation,
                                  obs->covariance);
        }
    }

    Pose2        pose() const override            { return ekf_.pose(); }
    BodyVelocity body_velocity() const override   { return ekf_.body_velocity(); }
    Matrix3      pose_covariance() const override { return ekf_.pose_covariance(); }

    void set_pose(Pose2 p, bool keep_velocity) override { ekf_.set_pose(p, keep_velocity); }
    void reset() override                                 { ekf_.reset(); }

private:
    Ekf ekf_;
};

// ── MCL adapter ─────────────────────────────────────────────────────────
// Tier 3: particle filter. MCL's predict consumes body-frame motion
// (forward, lateral, heading_delta) — the adapter computes heading_delta
// from successive heading_now arguments. process_observations() runs
// distance-sensor + landmark updates against the field map.
class MclLocalizer : public ILocalizer {
public:
    explicit MclLocalizer(Mcl::Config cfg,
                          Pose2  initial            = {},
                          double initial_xy_sigma   = 1.0,
                          double initial_h_sigma    = 0.1)
        : mcl_(std::move(cfg), initial, initial_xy_sigma, initial_h_sigma),
          prev_heading_(initial.heading) {}

    void predict(double par, double perp, Angle heading_now, double /*dt*/) override {
        const double dh = shortest_angle(prev_heading_, heading_now).rad;
        mcl_.predict(par, perp, dh);
        prev_heading_ = heading_now;
    }

    void process_observations(const Sensors& sensors) override {
        const auto distances = sensors.distance_sensors();
        if (!distances.empty()) mcl_.update_distance(distances);
        const auto landmarks = sensors.landmark_sensors();
        if (!landmarks.empty()) mcl_.update_landmarks(landmarks);
    }

    Pose2        pose() const override            { return mcl_.pose(); }
    BodyVelocity body_velocity() const override   { return BodyVelocity{}; }   // MCL is pose-only
    Matrix3      pose_covariance() const override { return mcl_.pose_covariance(); }

    void set_pose(Pose2 p, bool /*keep_velocity*/) override {
        mcl_.set_pose(p);
        prev_heading_ = p.heading;
    }
    void reset() override { mcl_.reset(); }

private:
    Mcl   mcl_;
    Angle prev_heading_;
};

} // namespace pathfinder
