#pragma once

#include <pathfinder/driving/interfaces.hpp>

// Host-side fake motor. Records the last voltage command (so tests can
// assert on it), and exposes setters for position / velocity / reversal so
// tests can drive the encoder feedback path.

namespace pathfinder {

class FakeMotor : public IMotor {
  public:
    void set_position(double revs) { pos_ = revs; }
    void set_velocity(double rpm)  { vel_ = rpm;  }
    void set_reversed(bool r)      { reversed_ = r; }

    // Inspection accessors for tests.
    double    last_voltage_mv() const { return last_voltage_mv_; }
    BrakeMode brake_mode()      const { return brake_; }

    void   set_voltage_mv(double mv) override { last_voltage_mv_ = mv; }
    double position_revolutions() const override { return reversed_ ? -pos_ : pos_; }
    double velocity_rpm() const override         { return reversed_ ? -vel_ : vel_; }
    void   set_brake_mode(BrakeMode mode) override { brake_ = mode; }
    bool   is_reversed() const override          { return reversed_; }
    void   reset_position(double revs = 0.0) override { pos_ = revs; }

  private:
    double    pos_              = 0.0;
    double    vel_              = 0.0;
    bool      reversed_         = false;
    double    last_voltage_mv_  = 0.0;
    BrakeMode brake_            = BrakeMode::Coast;
};

} // namespace pathfinder
