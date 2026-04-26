#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <optional>

// Host-side fakes implementing the sensor interfaces. Each fake exposes
// setters so a test can drive the underlying reading from outside, then make
// the consumer (DR / EKF / MCL / wheel adapter) read it back through the
// virtual API. No PROS, no hardware, no I/O.

namespace pathfinder {

class FakeRotation : public IRotation {
  public:
    void set_position(double revs)        { pos_      = revs; }
    void set_velocity(double rps)         { vel_      = rps;  }
    void set_reversed(bool reversed)      { reversed_ = reversed; }

    double position_revolutions() const override { return reversed_ ? -pos_ : pos_; }
    double velocity_rps() const override         { return reversed_ ? -vel_ : vel_; }
    void   reset_position(double new_revs = 0.0) override { pos_ = new_revs; }
    bool   is_reversed() const override          { return reversed_; }

  private:
    double pos_      = 0.0;
    double vel_      = 0.0;
    bool   reversed_ = false;
};

class FakeImu : public IImu {
  public:
    void set_heading(Angle h)              { heading_ = h; }
    void set_yaw_rate(double rad_s)        { yaw_rate_rad_s_ = rad_s; }
    void set_calibrating(bool c)           { calibrating_ = c; }

    Angle  heading_rad() const override       { return heading_; }
    double yaw_rate_rad_s() const override    { return yaw_rate_rad_s_; }
    bool   is_calibrating() const override    { return calibrating_; }
    void   reset_heading(Angle new_heading = Angle{}) override { heading_ = new_heading; }

  private:
    Angle  heading_{};
    double yaw_rate_rad_s_ = 0.0;
    bool   calibrating_    = false;
};

class FakeDistance : public IDistance {
  public:
    void set_distance(double inches) { distance_in_ = inches; }
    void set_valid(bool valid)       { valid_ = valid; }

    double distance_in() const override { return distance_in_; }
    bool   is_valid() const override    { return valid_; }

  private:
    double distance_in_ = 0.0;
    bool   valid_       = true;
};

class FakeLandmark : public ILandmark {
  public:
    void set_observation(LandmarkObservation obs) { obs_ = obs; }
    void clear_observation()                      { obs_.reset(); }

    std::optional<LandmarkObservation> poll() override { return obs_; }

  private:
    std::optional<LandmarkObservation> obs_;
};

} // namespace pathfinder
