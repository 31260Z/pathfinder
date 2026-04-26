#pragma once

#include <pathfinder/sensors/distance_sensor.hpp>
#include <pathfinder/sensors/imu.hpp>
#include <pathfinder/sensors/landmark_sensor_config.hpp>
#include <pathfinder/sensors/tracking_wheel.hpp>

#include <utility>
#include <vector>

namespace pathfinder {

// Declarative builder collecting every position-relevant sensor configured
// on the bot. Spec §6: "A single declarative sensor block describes
// everything that produces position-relevant data."
//
// The chassis layer (Wave C) consumes a `Sensors` instance and wires its
// contents into whichever localization tier the user selected (DR / EKF /
// MCL). Builder methods all return `*this` so calls chain.
class Sensors {
  public:
    Sensors& add(TrackingWheel wheel) {
        tracking_wheels_.push_back(std::move(wheel));
        return *this;
    }
    Sensors& add(Imu imu) {
        imus_.push_back(std::move(imu));
        return *this;
    }
    Sensors& add(DistanceSensor d) {
        distance_sensors_.push_back(std::move(d));
        return *this;
    }
    Sensors& add(LandmarkSensorConfig l) {
        landmark_sensors_.push_back(std::move(l));
        return *this;
    }

    const std::vector<TrackingWheel>&        tracking_wheels()   const { return tracking_wheels_; }
    const std::vector<Imu>&                  imus()              const { return imus_; }
    const std::vector<DistanceSensor>&       distance_sensors()  const { return distance_sensors_; }
    const std::vector<LandmarkSensorConfig>& landmark_sensors()  const { return landmark_sensors_; }

    // Convenience accessors: partition the tracking wheels by axis. Useful
    // for the odometry-DR config (one parallel + zero-or-one perpendicular).
    std::vector<TrackingWheel> parallel_wheels() const {
        std::vector<TrackingWheel> out;
        for (const auto& w : tracking_wheels_) {
            if (w.axis == Axis::X) out.push_back(w);
        }
        return out;
    }

    std::vector<TrackingWheel> perpendicular_wheels() const {
        std::vector<TrackingWheel> out;
        for (const auto& w : tracking_wheels_) {
            if (w.axis == Axis::Y) out.push_back(w);
        }
        return out;
    }

  private:
    std::vector<TrackingWheel>        tracking_wheels_;
    std::vector<Imu>                  imus_;
    std::vector<DistanceSensor>       distance_sensors_;
    std::vector<LandmarkSensorConfig> landmark_sensors_;
};

} // namespace pathfinder
