#pragma once

#include <pathfinder/driving/interfaces.hpp>

#include <cstddef>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

namespace pathfinder {

// Aggregate of `IMotor` shared pointers driven as a unit. The chassis layer
// constructs one MotorGroup per drivetrain side and commands voltage on it
// directly; aggregate position / velocity feed into the encoder-fallback
// branch of the odometry-DR pipeline (see spec §8 tier 1 sensor selection
// table).
//
// Aggregate position / velocity are **arithmetic means** of the per-motor
// readings. Each motor's `position_revolutions()` already accounts for its
// own reversal flag (see `IMotor::is_reversed`), so a same-side group of
// mixed-reversed motors averages out correctly without extra sign logic
// here. (The per-motor flag exists because real V5 builds often have
// adjacent motors mounted upside-down.)
class MotorGroup {
  public:
    MotorGroup(std::initializer_list<std::shared_ptr<IMotor>> motors)
        : motors_(motors) {}

    explicit MotorGroup(std::vector<std::shared_ptr<IMotor>> motors)
        : motors_(std::move(motors)) {}

    void set_voltage_mv(double mv) {
        for (auto& m : motors_) m->set_voltage_mv(mv);
    }

    void set_brake_mode(BrakeMode mode) {
        for (auto& m : motors_) m->set_brake_mode(mode);
    }

    double average_position_revolutions() const {
        if (motors_.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& m : motors_) sum += m->position_revolutions();
        return sum / static_cast<double>(motors_.size());
    }

    double average_velocity_rpm() const {
        if (motors_.empty()) return 0.0;
        double sum = 0.0;
        for (const auto& m : motors_) sum += m->velocity_rpm();
        return sum / static_cast<double>(motors_.size());
    }

    void reset_positions(double revs = 0.0) {
        for (auto& m : motors_) m->reset_position(revs);
    }

    std::size_t size() const { return motors_.size(); }

    const std::vector<std::shared_ptr<IMotor>>& motors() const { return motors_; }

  private:
    std::vector<std::shared_ptr<IMotor>> motors_;
};

} // namespace pathfinder
