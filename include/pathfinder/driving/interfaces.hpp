#pragma once

// Drivetrain motor abstraction. The chassis layer commands left/right motor
// groups via this interface; concrete adapters wrap PROS `pros::Motor` (Wave
// G) or, in tests, a `FakeMotor` that records the most recent voltage
// command so tests can assert on it.
//
// PROS-isolation: this header includes no `pros/...` headers. Voltage units
// are millivolts (±12000); position is wheel-shaft revolutions; velocity is
// shaft RPM.

namespace pathfinder {

enum class BrakeMode { Coast, Brake, Hold };

class IMotor {
  public:
    virtual ~IMotor() = default;

    virtual void   set_voltage_mv(double mv) = 0;       // ±12000
    virtual double position_revolutions() const = 0;
    virtual double velocity_rpm() const = 0;
    virtual void   set_brake_mode(BrakeMode mode) = 0;
    virtual bool   is_reversed() const = 0;
    virtual void   reset_position(double revs = 0.0) = 0;
};

} // namespace pathfinder
