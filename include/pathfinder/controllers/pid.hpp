#pragma once

#include <cmath>
#include <limits>

namespace pathfinder {

class Pid {
public:
    struct Gains {
        double kP = 0.0;
        double kI = 0.0;
        double kD = 0.0;
    };

    struct Limits {
        double output_max     = std::numeric_limits<double>::infinity();
        double integral_clamp = std::numeric_limits<double>::infinity();
        double deadband       = 0.0;
    };

    explicit Pid(Gains gains);
    Pid(Gains gains, Limits limits);

    double update(double setpoint, double measurement, double dt) {
        if (dt <= 0.0) {
            return last_output_;
        }

        double error = setpoint - measurement;
        if (std::abs(error) < limits_.deadband) {
            error = 0.0;
        }

        integral_ += error * dt;
        if (integral_ > limits_.integral_clamp) {
            integral_ = limits_.integral_clamp;
        } else if (integral_ < -limits_.integral_clamp) {
            integral_ = -limits_.integral_clamp;
        }

        double derivative = 0.0;
        if (!first_update_) {
            derivative = (error - prev_error_) / dt;
        }

        double output = gains_.kP * error
                      + gains_.kI * integral_
                      + gains_.kD * derivative;

        if (output > limits_.output_max) {
            output = limits_.output_max;
        } else if (output < -limits_.output_max) {
            output = -limits_.output_max;
        }

        prev_error_   = error;
        first_update_ = false;
        last_output_  = output;
        return output;
    }

    void reset() {
        integral_     = 0.0;
        prev_error_   = 0.0;
        last_output_  = 0.0;
        first_update_ = true;
    }

    Gains gains() const { return gains_; }
    void  set_gains(Gains g) { gains_ = g; }

    Limits limits() const { return limits_; }
    void   set_limits(Limits l) { limits_ = l; }

private:
    Gains  gains_;
    Limits limits_;
    double integral_     = 0.0;
    double prev_error_   = 0.0;
    double last_output_  = 0.0;
    bool   first_update_ = true;
};

inline Pid::Pid(Gains gains) : gains_(gains), limits_() {}
inline Pid::Pid(Gains gains, Limits limits) : gains_(gains), limits_(limits) {}

} // namespace pathfinder
