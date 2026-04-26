#pragma once

// PROS-side concrete IMotor adapter. Wraps `pros::Motor` and unit-converts
// per Pathfinder's interface contract:
//   set_voltage_mv(mv)        → motor_.move_voltage(int mv)
//   position_revolutions()    → motor_.get_position() degrees → revs
//   velocity_rpm()            → motor_.get_actual_velocity() (already RPM)
//   set_brake_mode(BrakeMode) → motor_.set_brake_mode(pros::E_MOTOR_BRAKE_*)
//   reset_position(revs)      → motor_.set_zero_position(degrees)
//
// PROS isolation: this header includes <pros/motors.hpp>; consequently it
// only compiles under a PROS toolchain. Host builds NEVER include this file.
//
// Reversal: PROS treats a negative port number as "reversed" at the
// constructor level. The adapter accepts a `bool reversed` flag and forwards
// `±port` to PROS so encoder readings come out with the right sign — and
// also flips `position_revolutions()` / `velocity_rpm()` defensively so the
// Pathfinder-visible reading always matches the IMotor contract regardless
// of whether the underlying PROS device flipped the sign for us.

#include <pathfinder/driving/interfaces.hpp>

#include <pros/motors.hpp>

#include <cmath>
#include <cstdint>

namespace pathfinder::pros_v5 {

class PROSMotor : public IMotor {
  public:
    explicit PROSMotor(int port, bool reversed = false)
        : motor_(static_cast<std::int8_t>(reversed ? -port : port)),
          reversed_(reversed) {
        // PROS defaults encoder units to degrees — that's what we expect for
        // get_position(). No explicit set_encoder_units call needed.
    }

    void set_voltage_mv(double mv) override {
        // PROS clamps internally; we still clamp here so a runaway double
        // never overflows the int conversion.
        if (mv >  12000.0) mv =  12000.0;
        if (mv < -12000.0) mv = -12000.0;
        const std::int32_t mv_i = static_cast<std::int32_t>(mv);
        motor_.move_voltage(mv_i);
    }

    double position_revolutions() const override {
        const double deg = motor_.get_position();   // PROS default unit
        const double revs = deg / 360.0;
        return reversed_ ? -revs : revs;
    }

    double velocity_rpm() const override {
        const double rpm = motor_.get_actual_velocity();
        return reversed_ ? -rpm : rpm;
    }

    void set_brake_mode(BrakeMode mode) override {
        pros::motor_brake_mode_e_t pm = pros::E_MOTOR_BRAKE_COAST;
        switch (mode) {
            case BrakeMode::Coast: pm = pros::E_MOTOR_BRAKE_COAST; break;
            case BrakeMode::Brake: pm = pros::E_MOTOR_BRAKE_BRAKE; break;
            case BrakeMode::Hold:  pm = pros::E_MOTOR_BRAKE_HOLD;  break;
        }
        motor_.set_brake_mode(pm);
    }

    bool is_reversed() const override { return reversed_; }

    void reset_position(double revs = 0.0) override {
        // PROS' set_zero_position takes the desired position value (degrees)
        // such that the next get_position() returns it. We want
        // position_revolutions() to read `revs` next call, so feed degrees.
        const double deg = (reversed_ ? -revs : revs) * 360.0;
        motor_.set_zero_position(deg);
    }

  private:
    pros::Motor motor_;
    bool        reversed_;
};

} // namespace pathfinder::pros_v5
