#pragma once

#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/geometry/angle.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace pathfinder::calibration {

enum class TuneAxis { Lateral, Angular };

// Open-loop step-response system identification + IMC PID gain synthesis.
// Spec §7.3 + Appendix B.
//
// Pipeline (per axis, lateral or angular):
//   Phase 1 — drive a bounded open-loop voltage step (forward for lateral,
//             differential for angular). Sample the body-frame velocity at
//             `sample_period_sec` for `step_duration + settle_period`
//             seconds (so we capture both rise and steady state).
//   Phase 2 — fit a first-order-plus-dead-time process model
//             `y(t) = K · (1 − e^{−(t−L)/τ})` to the response. Two-stage:
//               • K = mean of the late-time samples.
//               • For each rising-portion sample with
//                     0.05·K < y(t) < 0.95·K
//                 take y'(t) = ln(1 − y(t)/K) = (L − t) / τ, which is a
//                 linear function of t. OLS slope b = −1/τ; intercept a =
//                 L/τ. Solve τ = −1/b, L = a · τ. R² flags the fit
//                 quality.
//   Phase 3 — IMC gain synthesis with `λ = lambda_factor · τ`:
//               kP = (τ + L/2) / (K · (λ + L/2))
//               kI = kP / (τ + L/2)
//               kD = kP · τ · L / (2τ + L)
//   Phase 4 — verification: run a short closed-loop move with the new
//             gains; measure overshoot. If > threshold, increase `λ` by
//             50% and re-synthesize. Up to `max_verify_iterations` retries.
//
// The synthesized gains are clamped to conservative ranges before returning
// to keep a wonky fit (low R² but technically positive K, τ, L) from
// emitting numerically dangerous gains. Bounds:
//     kP ∈ [0.01, 100], kI ∈ [0, 50], kD ∈ [0, 10].
class AutoPid {
public:
    struct Spec {
        // Lateral step (both sides forward, fixed voltage).
        double lateral_step_voltage_mv  = 6000.0;
        double lateral_step_duration_sec = 1.0;
        // Angular step (one side forward, other side reverse).
        double angular_step_voltage_mv  = 4000.0;
        double angular_step_duration_sec = 0.7;
        // Common.
        double sample_period_sec        = 0.01;
        double settle_period_sec        = 1.5;     // observe steady state after the step ends
        // IMC closed-loop time constant: λ = lambda_factor · τ. λ = τ is
        // moderate (default); λ = 2τ is sluggish/safe; λ = τ/2 is
        // aggressive. Spec App. B.
        double lambda_factor            = 1.0;
        // Verification step magnitudes.
        double verify_lateral_distance_in = 12.0;
        double verify_angular_degrees    = 90.0;
        double overshoot_threshold_pct   = 15.0;
        int    max_verify_iterations     = 2;

        // Gain clamps (post-synthesis). Spec doesn't specify; we pick
        // conservative bounds and document. See `clamp_gains`.
        double kP_min = 0.01;
        double kP_max = 100.0;
        double kI_min = 0.0;
        double kI_max = 50.0;
        double kD_min = 0.0;
        double kD_max = 10.0;

        // FOPDT-fit acceptance threshold. Spec §7.3 says "if residual
        // variance is high, the tuner warns and recommends manual tuning."
        // We cash that out as: R² < min_r_squared ⇒ ok=false.
        double min_r_squared = 0.7;
    };

    struct ModelFit {
        double      K          = 0.0;   // steady-state gain (output_units / input_volts)
        double      tau        = 0.0;   // time constant (s)
        double      L          = 0.0;   // dead time (s)
        double      r_squared  = 0.0;
        bool        ok         = false;
        std::string message;
    };

    struct Result {
        Pid::Gains  gains{};
        ModelFit    model{};
        double      verified_overshoot_pct = 0.0;
        bool        converged              = false;
        std::string message;
    };

    explicit AutoPid(Chassis& chassis) : chassis_(chassis), spec_() {}
    AutoPid(Chassis& chassis, Spec spec) : chassis_(chassis), spec_(std::move(spec)) {}

    Result tune(TuneAxis axis) {
        Result r;
        if (spec_.sample_period_sec <= 0.0) {
            r.message = "AutoPid: sample_period_sec must be > 0";
            return r;
        }

        const double step_v   = (axis == TuneAxis::Lateral)
            ? spec_.lateral_step_voltage_mv
            : spec_.angular_step_voltage_mv;
        const double step_dur = (axis == TuneAxis::Lateral)
            ? spec_.lateral_step_duration_sec
            : spec_.angular_step_duration_sec;

        // ── Phase 1 — capture step response ────────────────────────────
        std::vector<double> ts;
        std::vector<double> ys;
        capture_step(axis, step_v, step_dur, ts, ys);

        // Make sure motors are off before we start any verification move.
        chassis_.applyVoltage(0.0, 0.0);

        // ── Phase 2 — FOPDT fit ────────────────────────────────────────
        r.model = fit_fopdt(ts, ys, spec_.min_r_squared);
        if (!r.model.ok) {
            r.message = "AutoPid: " + r.model.message
                      + " — recommend manual tuning.";
            return r;
        }

        // ── Phase 3 — IMC synthesis (with verification retry loop) ────
        double lambda_factor   = spec_.lambda_factor;
        Pid::Gains gains       = synthesize_imc(r.model, lambda_factor);
        gains                  = clamp_gains(gains);

        double overshoot_pct   = 0.0;
        bool   verified_ok     = false;
        for (int iter = 0; iter <= spec_.max_verify_iterations; ++iter) {
            overshoot_pct = run_verification(axis, gains);
            if (overshoot_pct <= spec_.overshoot_threshold_pct) {
                verified_ok = true;
                break;
            }
            if (iter == spec_.max_verify_iterations) break;
            // Too aggressive — slow down and re-synthesize.
            lambda_factor *= 1.5;
            gains          = clamp_gains(synthesize_imc(r.model, lambda_factor));
        }

        r.gains                  = gains;
        r.verified_overshoot_pct = overshoot_pct;
        r.converged              = verified_ok;
        if (verified_ok) {
            r.message = "AutoPid: tuned within overshoot threshold ("
                      + std::to_string(overshoot_pct) + "% <= "
                      + std::to_string(spec_.overshoot_threshold_pct) + "%).";
        } else {
            r.message = "AutoPid: verification overshoot ("
                      + std::to_string(overshoot_pct)
                      + "%) exceeds threshold after retries — gains returned"
                        " anyway, but consider manual tuning.";
        }
        return r;
    }

    std::pair<Result, Result> tuneBoth() {
        Result lat = tune(TuneAxis::Lateral);
        Result ang = tune(TuneAxis::Angular);
        return {lat, ang};
    }

    // For tests: a hook called every sample period BEFORE chassis.tick().
    // SimRig consumers register this so the simulated bot advances in
    // lock-step with the open-loop step capture.
    void set_pre_sample_hook(std::function<void(double t_sec)> hook) {
        pre_sample_hook_ = std::move(hook);
    }

private:
    void tick(double dt) {
        if (pre_sample_hook_) pre_sample_hook_(dt);
        chassis_.tick(dt);
    }

    // Phase 1 helper. Drives the open-loop step for `step_dur` seconds,
    // then coasts for `settle_period` seconds while sampling. Records
    // (t, v) into the supplied vectors. Coasting at the end gives us the
    // tail of the response so the steady-state estimate isn't dominated
    // by the rising portion.
    void capture_step(TuneAxis axis, double step_v, double step_dur,
                      std::vector<double>& ts, std::vector<double>& ys) {
        const double dt = spec_.sample_period_sec;
        const int n_step   = static_cast<int>(std::round(step_dur / dt));
        const int n_settle = static_cast<int>(std::round(spec_.settle_period_sec / dt));
        const int n_total  = n_step + n_settle;
        ts.reserve(static_cast<std::size_t>(n_total));
        ys.reserve(static_cast<std::size_t>(n_total));

        const double left_mv  = (axis == TuneAxis::Lateral) ?  step_v :  step_v;
        const double right_mv = (axis == TuneAxis::Lateral) ?  step_v : -step_v;

        for (int i = 0; i < n_step; ++i) {
            const double t = static_cast<double>(i) * dt;
            chassis_.applyVoltage(left_mv, right_mv);
            tick(dt);
            const auto vel = chassis_.getBodyVelocity();
            const double y = (axis == TuneAxis::Lateral) ? vel.v_x_ips : vel.omega_dps;
            ts.push_back(t);
            ys.push_back(y);
        }
        // Continue sampling past the step end so the steady-state estimate
        // covers the post-step coast tail. Note: for the FOPDT fit, the
        // samples we care about are those during the step (rising portion).
        for (int i = 0; i < n_settle; ++i) {
            const double t = static_cast<double>(n_step + i) * dt;
            chassis_.applyVoltage(0.0, 0.0);
            tick(dt);
            const auto vel = chassis_.getBodyVelocity();
            const double y = (axis == TuneAxis::Lateral) ? vel.v_x_ips : vel.omega_dps;
            ts.push_back(t);
            ys.push_back(y);
        }
        chassis_.applyVoltage(0.0, 0.0);
    }

    // Phase 2: fit FOPDT model y(t) = K(1 - exp(-(t - L)/τ)) using the
    // two-stage procedure from spec §7.3.
    //
    // K is estimated as the steady-state value reached during the step
    // (the late samples WITHIN the step duration, not the post-step coast
    // — coasting drops the velocity back toward zero so the post-step
    // mean would dilute K). We pick the last `tail_n` samples of the
    // RISING portion (i.e. before the response collapses). To keep this
    // a static helper, we use the maximum |y| seen as a robust proxy for
    // the step-end steady-state when K is unknown a priori.
    static ModelFit fit_fopdt(const std::vector<double>& ts,
                              const std::vector<double>& ys,
                              double min_r_squared) {
        ModelFit fit;
        if (ts.size() < 10 || ts.size() != ys.size()) {
            fit.message = "too few samples";
            return fit;
        }

        // Locate the index of the maximum |y|; samples around this index are
        // close to the step's steady state. We average the 10 samples
        // straddling the peak so a single noisy sample can't dominate K.
        std::size_t peak_idx = 0;
        double      peak_abs = 0.0;
        for (std::size_t i = 0; i < ys.size(); ++i) {
            const double a = std::abs(ys[i]);
            if (a > peak_abs) { peak_abs = a; peak_idx = i; }
        }
        if (peak_abs < 1e-9) {
            fit.message = "steady-state K ~= 0 — step had no effect";
            return fit;
        }
        // Average a small window around the peak for the K estimate. Bias
        // the window toward earlier samples (the response usually plateaus
        // and then immediately starts decaying once the step ends).
        const std::size_t window = std::min<std::size_t>(10, ys.size() / 4);
        const std::size_t lo = (peak_idx >= window) ? (peak_idx - window) : 0;
        const std::size_t hi = std::min(peak_idx + 1, ys.size());
        if (hi <= lo) {
            fit.message = "settle window empty";
            return fit;
        }
        double sum = 0.0;
        for (std::size_t i = lo; i < hi; ++i) sum += ys[i];
        const double K = sum / static_cast<double>(hi - lo);

        if (!std::isfinite(K) || std::abs(K) < 1e-9) {
            fit.message = "steady-state K ~= 0 — step had no effect";
            return fit;
        }

        // For each rising-portion sample where 0.05·K < y(t) < 0.95·K, take
        // y'(t) = ln(1 - y(t)/K) = (L - t) / τ. Linear regress against t.
        // Sign: K may be negative for an axis that decreases; normalize the
        // ratio to stay in the (0, 1) "rising" interval.
        const double K_abs = std::abs(K);
        std::vector<double> t_fit;
        std::vector<double> y_fit;
        t_fit.reserve(ts.size());
        y_fit.reserve(ts.size());
        for (std::size_t i = 0; i <= peak_idx && i < ts.size(); ++i) {
            const double ratio = ys[i] / K;       // ratio same sign as y/K
            if (ratio < 0.05 || ratio > 0.95) continue;
            const double arg = 1.0 - ratio;
            if (arg <= 0.0) continue;
            t_fit.push_back(ts[i]);
            y_fit.push_back(std::log(arg));
        }
        if (t_fit.size() < 3) {
            fit.message = "insufficient rising-portion samples (response too "
                          "abrupt or too noisy)";
            fit.K = K_abs;
            return fit;
        }

        // OLS: y = a + b·t with b = -1/τ and a = L/τ.
        const double n = static_cast<double>(t_fit.size());
        double sum_x = 0.0, sum_y = 0.0;
        for (std::size_t i = 0; i < t_fit.size(); ++i) {
            sum_x += t_fit[i];
            sum_y += y_fit[i];
        }
        const double mean_x = sum_x / n;
        const double mean_y = sum_y / n;
        double s_xx = 0.0, s_xy = 0.0, s_yy = 0.0;
        for (std::size_t i = 0; i < t_fit.size(); ++i) {
            const double dx = t_fit[i] - mean_x;
            const double dy = y_fit[i] - mean_y;
            s_xx += dx * dx;
            s_xy += dx * dy;
            s_yy += dy * dy;
        }
        if (s_xx <= 0.0) {
            fit.message = "degenerate fit window (all samples at one t)";
            return fit;
        }
        const double slope     = s_xy / s_xx;       // = -1/τ
        const double intercept = mean_y - slope * mean_x; // = L/τ
        const double r2 = (s_yy > 0.0) ? ((s_xy * s_xy) / (s_xx * s_yy)) : 0.0;

        if (slope >= 0.0) {
            fit.K       = K_abs;
            fit.r_squared = r2;
            fit.message = "fitted slope is non-negative — process not first-order rising";
            return fit;
        }

        const double tau = -1.0 / slope;
        const double L   = intercept * tau;

        fit.K          = K_abs;
        fit.tau        = tau;
        fit.L          = std::max(0.0, L);   // negative dead time isn't physical
        fit.r_squared  = r2;
        if (r2 < min_r_squared || tau <= 0.0) {
            fit.ok      = false;
            fit.message = "low R^2 (" + std::to_string(r2) + ") or non-positive tau "
                          "— process may not be FOPDT (resonance / backlash)";
            return fit;
        }
        fit.ok      = true;
        fit.message = "FOPDT fit ok (R^2 = " + std::to_string(r2) + ")";
        return fit;
    }

    static Pid::Gains synthesize_imc(const ModelFit& m, double lambda_factor) {
        // λ = lambda_factor · τ. Spec App. B.
        const double lambda = lambda_factor * m.tau;
        const double half_L = 0.5 * m.L;
        const double Ti     = m.tau + half_L;
        Pid::Gains g{};
        // Guard against pathological (K, λ + L/2) configurations.
        const double denom_kP = m.K * (lambda + half_L);
        if (denom_kP <= 0.0 || !std::isfinite(denom_kP)) return g;
        g.kP = Ti / denom_kP;
        g.kI = (Ti > 0.0) ? (g.kP / Ti) : 0.0;
        const double Td_denom = 2.0 * m.tau + m.L;
        g.kD = (Td_denom > 0.0) ? (g.kP * (m.tau * m.L) / Td_denom) : 0.0;
        return g;
    }

    Pid::Gains clamp_gains(Pid::Gains g) const {
        g.kP = std::clamp(g.kP, spec_.kP_min, spec_.kP_max);
        g.kI = std::clamp(g.kI, spec_.kI_min, spec_.kI_max);
        g.kD = std::clamp(g.kD, spec_.kD_min, spec_.kD_max);
        return g;
    }

    // Phase 4 — verify. Run a short closed-loop move with the new gains and
    // measure the overshoot in % relative to the target magnitude.
    //
    // We do this WITHOUT reaching for `chassis.moveTo` / `turnTo` — those
    // would consume the full controller stack (cross-track, motion profile,
    // …) and obscure what `gains` did. Instead, we drive a single PID
    // directly against the relevant scalar setpoint (distance traveled or
    // heading change since the start of the verification move) and watch the
    // peak. This isolates the gains from the rest of the autonomous stack
    // and is what the spec text really meant by "verification" — a tight
    // feedback loop that reports the resulting overshoot.
    double run_verification(TuneAxis axis, Pid::Gains gains) {
        // Re-zero the chassis pose so we measure progress from 0.
        chassis_.setPose(0.0, 0.0, 0.0);

        const double target = (axis == TuneAxis::Lateral)
            ? spec_.verify_lateral_distance_in
            : spec_.verify_angular_degrees;

        Pid::Limits limits;
        limits.output_max     = 12000.0;
        limits.integral_clamp = 5000.0;
        Pid pid(gains, limits);

        const double dt = spec_.sample_period_sec;
        const double timeout_sec = (axis == TuneAxis::Lateral) ? 3.0 : 2.0;
        const int    n_ticks     = static_cast<int>(std::round(timeout_sec / dt));

        double max_progress = 0.0;
        for (int i = 0; i < n_ticks; ++i) {
            const auto pose = chassis_.getPose();
            const double progress = (axis == TuneAxis::Lateral)
                ? std::sqrt(pose.x * pose.x + pose.y * pose.y)
                : std::abs(pose.heading.rad * k_rad_to_deg);
            if (progress > max_progress) max_progress = progress;

            const double output = pid.update(target, progress, dt);
            const double mv     = std::clamp(output, -12000.0, 12000.0);
            if (axis == TuneAxis::Lateral) {
                chassis_.applyVoltage(mv, mv);
            } else {
                chassis_.applyVoltage(mv, -mv);
            }
            tick(dt);
        }
        chassis_.applyVoltage(0.0, 0.0);

        const double overshoot_abs = std::max(0.0, max_progress - std::abs(target));
        const double pct = (std::abs(target) > 1e-9)
            ? (overshoot_abs / std::abs(target)) * 100.0
            : 0.0;
        return pct;
    }

    Chassis& chassis_;
    Spec     spec_;
    std::function<void(double t_sec)> pre_sample_hook_;
};

} // namespace pathfinder::calibration
