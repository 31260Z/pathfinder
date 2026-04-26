#pragma once

namespace pathfinder {

// First call snaps to target (no rate limit on initial setup; the limiter
// caps subsequent *changes*). Call reset(initial_value) before update() to
// start from a specific non-target value.
class SlewLimiter {
public:
    explicit SlewLimiter(double max_rate_per_sec)
        : max_rate_(max_rate_per_sec) {}

    double update(double target, double dt_sec) {
        if (first_) {
            current_ = target;
            first_   = false;
            return current_;
        }
        if (dt_sec <= 0.0) {
            return current_;
        }

        const double max_step = max_rate_ * dt_sec;
        const double delta    = target - current_;
        if (delta > max_step) {
            current_ += max_step;
        } else if (delta < -max_step) {
            current_ -= max_step;
        } else {
            current_ = target;
        }
        return current_;
    }

    void reset(double initial_value = 0.0) {
        current_ = initial_value;
        first_   = false;
    }

    double current() const { return current_; }
    double max_rate() const { return max_rate_; }
    void   set_max_rate(double r) { max_rate_ = r; }

private:
    double max_rate_;
    double current_ = 0.0;
    bool   first_   = true;
};

} // namespace pathfinder
