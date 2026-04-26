#pragma once

#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace pathfinder::calibration {

// Auto-measures the lateral friction coefficient `μ` (units: 1/s) used by
// the spec §8 v_y process model. Per spec §9 ("Drift-aware control"), the
// procedure is:
//
//   Phase 1 — accelerate forward at a known voltage for a short time so the
//             bot picks up forward velocity (v_x).
//   Phase 2 — apply a brief differential voltage to abruptly rotate the
//             bot. The cross-coupling term `−ω · v_x` injects lateral
//             velocity (v_y) into the body frame.
//   Phase 3 — coast (zero voltage). v_y decays as `v_y(t) = v_y_0 · e^{−μt}`.
//             Sample the chassis's body_velocity() each tick.
//   Fit    — linear regression on `ln(|v_y(t)|)` vs `t`. The slope is `−μ`;
//             R² indicates fit quality. Skip the first few samples to avoid
//             the rotation-tail transient.
//
// Notes:
//   • The decay is observed via chassis.getBodyVelocity().v_y_ips. With no
//     perpendicular tracking wheel, v_y is forward-Euler-integrated by the
//     DR — same model we're trying to fit, so the recovered μ matches the
//     model's μ given consistent bot dynamics. With a perp wheel, v_y is a
//     direct measurement and the fit reflects physical reality.
//   • If R² < 0.7, we mark the result not-converged and recommend a stronger
//     rotation phase or installing a perpendicular tracking wheel.
class DriftCoeff {
public:
    struct Spec {
        double forward_voltage_mv       = 8000.0;   // Phase 1 magnitude
        double forward_duration_sec     = 0.5;      // accelerate to known v_x
        double rotation_voltage_mv      = 6000.0;   // Phase 2 differential
        double rotation_duration_sec    = 0.3;      // brief rotation to inject v_y
        double observation_duration_sec = 1.5;      // Phase 3 — watch decay
        double sample_period_sec        = 0.01;     // 100 Hz tick
        // How many initial Phase-3 samples to discard (rotation-tail
        // transient — the bot's actual v_y rebound takes a few ticks to
        // settle after the rotation phase ends).
        std::size_t skip_initial_samples = 5;
        // Minimum |v_y| threshold (ips) for samples to enter the regression.
        // Below this, ln-of-noise dominates and the fit explodes. Default
        // 0.05 ips ≈ 1.3 mm/s; effectively zero on a real bot.
        double min_abs_v_y_for_fit_ips  = 0.05;
        // R² below this marks the result as not-converged.
        double min_r_squared            = 0.7;
        // Pre-tick hook (optional). Called once per cycle BEFORE
        // chassis.tick() so a host-side simulator can advance the bot
        // physics in lock-step with the calibration loop. On real hardware
        // this is left null and the bot moves continuously between ticks via
        // the actual motors. Default: no-op.
        std::function<void(double dt_sec)> pre_tick_hook;
    };

    struct Result {
        double      mu_per_sec  = 0.0;     // fitted lateral_friction_coefficient
        double      r_squared   = 0.0;     // fit quality, 0..1 (higher = better)
        bool        converged   = false;   // false → result not trustworthy
        std::string message;               // human-readable explanation
        // Diagnostics — exposed for tests and for the runtime tuning UI.
        double      v_y_initial_ips = 0.0; // |v_y| at first kept sample
        std::size_t samples_used    = 0;   // count entering regression
    };

    explicit DriftCoeff(Chassis& chassis) : chassis_(chassis), spec_() {}
    DriftCoeff(Chassis& chassis, Spec spec)
        : chassis_(chassis), spec_(std::move(spec)) {}

    Result measure() {
        Result r;
        if (spec_.sample_period_sec <= 0.0) {
            r.message = "DriftCoeff: sample_period_sec must be > 0";
            return r;
        }

        const int  ticks_phase1 = static_cast<int>(
            std::round(spec_.forward_duration_sec   / spec_.sample_period_sec));
        const int  ticks_phase2 = static_cast<int>(
            std::round(spec_.rotation_duration_sec  / spec_.sample_period_sec));
        const int  ticks_phase3 = static_cast<int>(
            std::round(spec_.observation_duration_sec / spec_.sample_period_sec));

        // Phase 1 — accelerate forward.
        for (int i = 0; i < ticks_phase1; ++i) {
            chassis_.applyVoltage(spec_.forward_voltage_mv, spec_.forward_voltage_mv);
            tick();
        }

        // Phase 2 — abrupt rotation while still moving forward. Keep the
        // forward voltage on top of the differential so v_x remains high
        // (the cross-coupling term is `−ω · v_x`; if v_x ≈ 0 there's no
        // injection). Left side faster than right ⇒ CW (positive ω) in our
        // tank kinematics convention.
        const double left_mv  = spec_.forward_voltage_mv + spec_.rotation_voltage_mv;
        const double right_mv = spec_.forward_voltage_mv - spec_.rotation_voltage_mv;
        for (int i = 0; i < ticks_phase2; ++i) {
            chassis_.applyVoltage(left_mv, right_mv);
            tick();
        }

        // Phase 3 — coast and sample v_y(t).
        chassis_.applyVoltage(0.0, 0.0);
        std::vector<double> ts;
        std::vector<double> v_ys;
        ts.reserve(static_cast<std::size_t>(ticks_phase3));
        v_ys.reserve(static_cast<std::size_t>(ticks_phase3));
        const double dt = spec_.sample_period_sec;
        for (int i = 0; i < ticks_phase3; ++i) {
            chassis_.applyVoltage(0.0, 0.0);
            tick();
            ts.push_back(static_cast<double>(i) * dt);
            v_ys.push_back(chassis_.getBodyVelocity().v_y_ips);
        }
        chassis_.applyVoltage(0.0, 0.0);

        // Fit ln(|v_y|) = ln(|v_y_0|) − μ·t. Drop the rotation-tail
        // transient at the start, then drop any sample whose |v_y| is
        // below the noise floor.
        std::vector<double> t_fit, lnv_fit;
        t_fit.reserve(v_ys.size());
        lnv_fit.reserve(v_ys.size());
        for (std::size_t i = spec_.skip_initial_samples; i < v_ys.size(); ++i) {
            const double abs_v = std::abs(v_ys[i]);
            if (abs_v < spec_.min_abs_v_y_for_fit_ips) continue;
            t_fit.push_back(ts[i]);
            lnv_fit.push_back(std::log(abs_v));
        }
        r.samples_used = t_fit.size();

        if (t_fit.size() < 3) {
            r.message = "DriftCoeff: too few usable samples — increase the "
                        "rotation phase voltage/duration, or add a perpendicular "
                        "tracking wheel for direct measurement.";
            return r;
        }

        // OLS: y = a + b·x with x=t, y=ln|v_y|. Slope b = −μ.
        const double n = static_cast<double>(t_fit.size());
        double sum_x = 0.0, sum_y = 0.0;
        for (std::size_t i = 0; i < t_fit.size(); ++i) {
            sum_x += t_fit[i];
            sum_y += lnv_fit[i];
        }
        const double mean_x = sum_x / n;
        const double mean_y = sum_y / n;

        double s_xx = 0.0, s_xy = 0.0, s_yy = 0.0;
        for (std::size_t i = 0; i < t_fit.size(); ++i) {
            const double dx = t_fit[i]    - mean_x;
            const double dy = lnv_fit[i] - mean_y;
            s_xx += dx * dx;
            s_xy += dx * dy;
            s_yy += dy * dy;
        }
        if (s_xx <= 0.0) {
            r.message = "DriftCoeff: degenerate time axis (all samples at one t)";
            return r;
        }
        const double slope = s_xy / s_xx;        // = −μ
        // R² = (s_xy)^2 / (s_xx · s_yy) for OLS slope; 0 when s_yy is 0
        // (constant y, perfect noise-floor — but we'd have already early-
        // returned in that case).
        const double r2 = (s_yy > 0.0) ? ((s_xy * s_xy) / (s_xx * s_yy)) : 0.0;

        r.mu_per_sec      = -slope;
        r.r_squared       = r2;
        r.v_y_initial_ips = std::exp(lnv_fit.front());
        if (r.mu_per_sec <= 0.0 || !std::isfinite(r.mu_per_sec)) {
            r.message = "DriftCoeff: fitted μ is non-positive or non-finite — "
                        "rotation phase may have been too gentle to inject "
                        "measurable v_y. Increase rotation_voltage_mv or duration.";
            return r;
        }
        if (r2 < spec_.min_r_squared) {
            r.message = "DriftCoeff: low R² (" + std::to_string(r2)
                      + ") — increase the rotation phase voltage/duration, "
                        "or add a perpendicular tracking wheel for direct "
                        "measurement.";
            return r;
        }
        r.converged = true;
        r.message   = "DriftCoeff: μ = " + std::to_string(r.mu_per_sec)
                    + " /s (R² = " + std::to_string(r2) + ")";
        return r;
    }

private:
    void tick() {
        if (spec_.pre_tick_hook) spec_.pre_tick_hook(spec_.sample_period_sec);
        chassis_.tick(spec_.sample_period_sec);
    }

    Chassis& chassis_;
    Spec     spec_;
};

} // namespace pathfinder::calibration
