#pragma once

#include <limits>

namespace pathfinder {

class ExitConditions {
public:
    struct Spec {
        double small_error            = 0.0;
        double small_error_timeout_ms = 0.0;
        double large_error            = 0.0;
        double large_error_timeout_ms = 0.0;
        double absolute_timeout_ms    = std::numeric_limits<double>::infinity();
    };

    explicit ExitConditions(Spec spec) : spec_(spec) {}

    void feed(double error_abs, double dt_ms) {
        total_elapsed_ms_ += dt_ms;

        if (error_abs <= spec_.small_error) {
            small_band_held_ms_ += dt_ms;
        } else {
            small_band_held_ms_ = 0.0;
        }

        if (error_abs <= spec_.large_error) {
            large_band_held_ms_ += dt_ms;
        } else {
            large_band_held_ms_ = 0.0;
        }
    }

    void reset() {
        small_band_held_ms_ = 0.0;
        large_band_held_ms_ = 0.0;
        total_elapsed_ms_   = 0.0;
    }

    bool is_done() const {
        if (total_elapsed_ms_ >= spec_.absolute_timeout_ms) return true;
        if (small_band_held_ms_ >= spec_.small_error_timeout_ms
            && spec_.small_error_timeout_ms > 0.0) return true;
        if (large_band_held_ms_ >= spec_.large_error_timeout_ms
            && spec_.large_error_timeout_ms > 0.0) return true;
        return false;
    }

    Spec   spec() const { return spec_; }
    double small_band_held_ms() const { return small_band_held_ms_; }
    double large_band_held_ms() const { return large_band_held_ms_; }
    double total_elapsed_ms() const { return total_elapsed_ms_; }

private:
    Spec   spec_;
    double small_band_held_ms_ = 0.0;
    double large_band_held_ms_ = 0.0;
    double total_elapsed_ms_   = 0.0;
};

} // namespace pathfinder
