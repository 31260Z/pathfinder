#pragma once

#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/sensors/tracking_wheel.hpp>

#include <cmath>
#include <cstddef>
#include <functional>
#include <string>
#include <utility>
#include <vector>

namespace pathfinder::calibration {

// Auto-finder for tracking-wheel offsets — spec §7.2 + Appendix A.
//
// The user pins a known corner of the bot (the "pivot point" P), manually
// rotates it through ~360°, and we read accumulated IMU heading change
// `Δθ` plus accumulated encoder distance `Δs` for each tracking wheel.
//
// Per spec App. A:
//   • Parallel  wheel reads `Δs_par  = (P_y − y_p) · Δθ`
//                     ⟹ `y_p = P_y − Δs_par  / Δθ`
//   • Perp      wheel reads `Δs_perp = (x_q − P_x) · Δθ`
//                     ⟹ `x_q = P_x + Δs_perp / Δθ`
//
// `P` is the pivot point in **bot-frame** coordinates (read from
// `Bot::footprint().corners(origin)`).
//
// The library does NOT command the motors during a pivot — the user spins
// the bot manually. We just sample the IMU heading and encoders. The pivot
// run terminates when:
//   (a) the IMU yaw rate drops below `still_yaw_rate_dps_` for at least
//       `still_dwell_sec_` seconds AND `|Δθ| ≥ min_rotation_rad`, OR
//   (b) `timeout_per_pivot_sec` elapses.
class WheelFinder {
public:
    enum class PivotCorner { FrontLeft, FrontRight, BackLeft, BackRight };

    struct Spec {
        // Minimum heading change required for a single pivot run to count.
        // Below this, the noise floor on IMU + encoder dominates the recovered
        // offset. Default ~3 rad (~172°) — better than the bare 90° threshold
        // many tape-measure setups would tolerate, but doesn't strictly
        // require a full revolution if the bot is tethered or in a tight
        // space.
        double min_rotation_rad = 3.0;
        double sample_period_sec = 0.01;
        // Maximum allowed wall-time per pivot (the user might give up).
        double timeout_per_pivot_sec = 60.0;

        // "User has stopped rotating" detector — the pivot completes when
        // |yaw_rate| stays below this threshold for `still_dwell_sec`
        // continuously, AND we've already swept at least `min_rotation_rad`.
        double still_yaw_rate_dps  = 5.0;
        double still_dwell_sec     = 0.5;
    };

    struct PivotResult {
        PivotCorner       corner = PivotCorner::BackLeft;
        double            delta_theta_rad = 0.0;   // accumulated IMU heading change
        // Per-wheel estimates from this pivot (input order):
        std::vector<double> parallel_y_estimates_in;
        std::vector<double> perpendicular_x_estimates_in;
        bool              converged = false;
        std::string       message;
    };

    struct AggregateResult {
        struct PerWheel {
            double      mean_offset_in   = 0.0;
            double      stddev_offset_in = 0.0;
            unsigned    n_samples        = 0;
        };
        std::vector<PerWheel>     parallel_wheels;        // matches Sensors::parallel_wheels() order
        std::vector<PerWheel>     perpendicular_wheels;
        std::vector<PivotResult>  per_pivot_raw;          // for debugging
        bool                      converged = false;       // all wheels have ≥ 1 valid sample
        std::string               message;
    };

    explicit WheelFinder(Chassis& chassis) : chassis_(chassis), spec_() {}
    WheelFinder(Chassis& chassis, Spec spec)
        : chassis_(chassis), spec_(std::move(spec)) {}

    // Run a single pivot. The user must manually pin the named corner and
    // rotate the bot before calling this. The function blocks while sampling.
    // We do NOT command the motors — `applyVoltage(0, 0)` once at start so
    // the motors are at coast.
    PivotResult run_pivot(PivotCorner corner) {
        PivotResult result;
        result.corner = corner;

        // Cache the pivot point in bot-frame.
        const Vector2 pivot = corner_in_bot_frame(corner);

        // Snapshot the per-wheel start positions and the IMU heading.
        const auto par_wheels  = chassis_.sensors().parallel_wheels();
        const auto perp_wheels = chassis_.sensors().perpendicular_wheels();

        std::vector<double> par_start_revs;
        std::vector<double> perp_start_revs;
        par_start_revs.reserve(par_wheels.size());
        perp_start_revs.reserve(perp_wheels.size());
        for (const auto& w : par_wheels)  par_start_revs.push_back(read_revs(w));
        for (const auto& w : perp_wheels) perp_start_revs.push_back(read_revs(w));

        const double start_heading_rad = chassis_.getPose().heading.rad;
        double prev_heading_rad        = start_heading_rad;
        double accumulated_delta_theta = 0.0;

        // Ensure the motors are coasted at the entry of the pivot — the user
        // is the one rotating the bot.
        chassis_.applyVoltage(0.0, 0.0);

        const double dt = spec_.sample_period_sec;
        if (dt <= 0.0) {
            result.message = "WheelFinder: sample_period_sec must be > 0";
            return result;
        }

        const int max_ticks = static_cast<int>(
            std::ceil(spec_.timeout_per_pivot_sec / dt));
        double still_dwell_accum_sec = 0.0;
        bool   still_user_done       = false;

        for (int tick = 0; tick < max_ticks; ++tick) {
            if (pre_sample_hook_) pre_sample_hook_(static_cast<double>(tick) * dt);
            chassis_.tick(dt);

            // Integrate Δθ as a sum of `shortest_angle` deltas so we can
            // observe rotations greater than π (naive end-minus-start would
            // wrap on a >180° sweep).
            const double now_heading_rad = chassis_.getPose().heading.rad;
            const double tick_d_theta    = shortest_angle(
                Angle{prev_heading_rad}, Angle{now_heading_rad}).rad;
            accumulated_delta_theta += tick_d_theta;
            prev_heading_rad         = now_heading_rad;

            // Yaw-rate idle detector. Use the per-tick Δθ / dt to estimate
            // the rate (the IMU's own velocity reading might not be wired
            // up consistently across the host fakes).
            const double inst_rate_dps = std::abs(tick_d_theta / dt) * k_rad_to_deg;
            if (inst_rate_dps < spec_.still_yaw_rate_dps) {
                still_dwell_accum_sec += dt;
            } else {
                still_dwell_accum_sec = 0.0;
            }

            if (still_dwell_accum_sec >= spec_.still_dwell_sec
                && std::abs(accumulated_delta_theta) >= spec_.min_rotation_rad) {
                still_user_done = true;
                break;
            }
        }

        result.delta_theta_rad = accumulated_delta_theta;
        const double abs_d_theta = std::abs(accumulated_delta_theta);
        if (abs_d_theta < spec_.min_rotation_rad) {
            result.converged = false;
            result.message   = "WheelFinder: insufficient rotation ("
                             + std::to_string(abs_d_theta)
                             + " rad < " + std::to_string(spec_.min_rotation_rad)
                             + " rad). Spin the bot through more of a revolution.";
            return result;
        }

        // Solve App. A for each wheel and emit estimates in input order.
        result.parallel_y_estimates_in.reserve(par_wheels.size());
        for (std::size_t i = 0; i < par_wheels.size(); ++i) {
            const double end_revs = read_revs(par_wheels[i]);
            const double d_revs   = end_revs - par_start_revs[i];
            const double d_in     = par_wheels[i].encoder_to_inches(d_revs);
            // y_p = P_y − Δs_par / Δθ
            const double y_p = pivot.y - (d_in / accumulated_delta_theta);
            result.parallel_y_estimates_in.push_back(y_p);
        }
        result.perpendicular_x_estimates_in.reserve(perp_wheels.size());
        for (std::size_t i = 0; i < perp_wheels.size(); ++i) {
            const double end_revs = read_revs(perp_wheels[i]);
            const double d_revs   = end_revs - perp_start_revs[i];
            const double d_in     = perp_wheels[i].encoder_to_inches(d_revs);
            // x_q = P_x + Δs_perp / Δθ
            const double x_q = pivot.x + (d_in / accumulated_delta_theta);
            result.perpendicular_x_estimates_in.push_back(x_q);
        }

        result.converged = true;
        result.message   = still_user_done
            ? "WheelFinder: pivot complete (user idle detected)."
            : "WheelFinder: pivot complete (timeout reached, but rotation "
              "was sufficient).";
        return result;
    }

    // Convenience: run all four corners in sequence and aggregate the
    // results. The caller is responsible for prompting the user to
    // reposition between pivots; on host tests the `pre_sample_hook`
    // simulator advances the synthetic bot in lock-step.
    AggregateResult run_all_corners() {
        AggregateResult agg;

        // Iteration order: BackLeft → FrontLeft → FrontRight → BackRight.
        // (Sweeps around the bot — feels natural for a user repositioning
        // their finger between pivots.)
        constexpr PivotCorner k_order[] = {
            PivotCorner::BackLeft, PivotCorner::FrontLeft,
            PivotCorner::FrontRight, PivotCorner::BackRight,
        };

        for (PivotCorner c : k_order) {
            // Make sure the motors are off between pivots.
            chassis_.applyVoltage(0.0, 0.0);
            agg.per_pivot_raw.push_back(run_pivot(c));
        }

        // Aggregate per wheel. We size the per-wheel vectors from the first
        // converged pivot (every pivot iterates the same wheel list, just
        // some may have failed to converge — those don't contribute).
        const std::size_t n_par  = chassis_.sensors().parallel_wheels().size();
        const std::size_t n_perp = chassis_.sensors().perpendicular_wheels().size();
        agg.parallel_wheels.assign(n_par, {});
        agg.perpendicular_wheels.assign(n_perp, {});

        // Walk every pivot's per-wheel estimates and stream them into a
        // running mean / stddev.
        std::vector<std::vector<double>> par_samples(n_par);
        std::vector<std::vector<double>> perp_samples(n_perp);
        for (const auto& pr : agg.per_pivot_raw) {
            if (!pr.converged) continue;
            for (std::size_t i = 0;
                 i < pr.parallel_y_estimates_in.size() && i < n_par; ++i) {
                par_samples[i].push_back(pr.parallel_y_estimates_in[i]);
            }
            for (std::size_t i = 0;
                 i < pr.perpendicular_x_estimates_in.size() && i < n_perp; ++i) {
                perp_samples[i].push_back(pr.perpendicular_x_estimates_in[i]);
            }
        }

        bool all_have_samples = (n_par + n_perp) > 0;
        for (std::size_t i = 0; i < n_par; ++i) {
            agg.parallel_wheels[i] = summarize(par_samples[i]);
            if (agg.parallel_wheels[i].n_samples == 0) all_have_samples = false;
        }
        for (std::size_t i = 0; i < n_perp; ++i) {
            agg.perpendicular_wheels[i] = summarize(perp_samples[i]);
            if (agg.perpendicular_wheels[i].n_samples == 0) all_have_samples = false;
        }

        agg.converged = all_have_samples;
        agg.message   = agg.converged
            ? "WheelFinder: all wheels have at least one valid pivot sample."
            : "WheelFinder: at least one wheel has zero converged pivot "
              "samples — re-run any failed corners.";
        return agg;
    }

    // For tests: a hook called every sample period BEFORE chassis.tick(),
    // letting a SimRig advance its simulated physics in lock-step with the
    // sampling loop. Real hardware leaves this null.
    void set_pre_sample_hook(std::function<void(double t_sec)> hook) {
        pre_sample_hook_ = std::move(hook);
    }

private:
    static double read_revs(const TrackingWheel& w) {
        return w.sensor ? w.sensor->position_revolutions() : 0.0;
    }

    // Resolve a `PivotCorner` into bot-frame `(x, y)` using the bot's
    // declared footprint and chosen origin corner.
    Vector2 corner_in_bot_frame(PivotCorner pc) const {
        const auto corners = chassis_.bot().footprint().corners(chassis_.bot().origin());
        // Footprint::corners returns FrontLeft, FrontRight, BackLeft, BackRight.
        switch (pc) {
            case PivotCorner::FrontLeft:  return corners[0];
            case PivotCorner::FrontRight: return corners[1];
            case PivotCorner::BackLeft:   return corners[2];
            case PivotCorner::BackRight:  return corners[3];
        }
        return Vector2{};
    }

    static AggregateResult::PerWheel summarize(const std::vector<double>& samples) {
        AggregateResult::PerWheel out;
        out.n_samples = static_cast<unsigned>(samples.size());
        if (samples.empty()) return out;
        double sum = 0.0;
        for (double s : samples) sum += s;
        out.mean_offset_in = sum / static_cast<double>(samples.size());
        if (samples.size() < 2) {
            out.stddev_offset_in = 0.0;
            return out;
        }
        double sq_sum = 0.0;
        for (double s : samples) {
            const double d = s - out.mean_offset_in;
            sq_sum += d * d;
        }
        out.stddev_offset_in = std::sqrt(sq_sum / static_cast<double>(samples.size() - 1));
        return out;
    }

    Chassis&                              chassis_;
    Spec                                  spec_;
    std::function<void(double t_sec)>     pre_sample_hook_;
};

} // namespace pathfinder::calibration
