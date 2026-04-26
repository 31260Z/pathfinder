#pragma once

#include <pathfinder/geometry/vector2.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

namespace pathfinder {
namespace catmull_rom {

struct ControlPoint {
    Vector2               position{};
    std::optional<double> tangent_speed_hint = std::nullopt;
};

class Path {
public:
    explicit Path(std::vector<Vector2> waypoints, double tension = 0.5)
        : waypoints_(std::move(waypoints)),
          alpha_(tension) {
        if (waypoints_.size() < 2) {
            throw std::invalid_argument("catmull_rom::Path needs >= 2 waypoints");
        }
        build_segments();
        discretize();
        compute_waypoint_arclengths();
    }

    Vector2 sample(double t) const {
        if (samples_.empty()) return waypoints_.front();
        if (t <= 0.0) return samples_.front();
        if (t >= 1.0) return samples_.back();
        const double idx_f = t * static_cast<double>(samples_.size() - 1);
        const std::size_t i0 = static_cast<std::size_t>(std::floor(idx_f));
        const std::size_t i1 = std::min(i0 + 1, samples_.size() - 1);
        const double frac = idx_f - static_cast<double>(i0);
        return samples_[i0] * (1.0 - frac) + samples_[i1] * frac;
    }

    Vector2 sample_at_arclength(double s_in) const {
        const std::size_t i = arclength_index(s_in);
        if (i + 1 >= samples_.size()) return samples_.back();
        const double s0 = cum_arclength_[i];
        const double s1 = cum_arclength_[i + 1];
        const double seg = s1 - s0;
        const double frac = (seg > 1e-12) ? (s_in - s0) / seg : 0.0;
        return samples_[i] * (1.0 - frac) + samples_[i + 1] * frac;
    }

    double total_length_in() const {
        return cum_arclength_.empty() ? 0.0 : cum_arclength_.back();
    }

    Vector2 tangent_at_arclength(double s_in) const {
        if (samples_.size() < 2) return Vector2{1.0, 0.0};
        const std::size_t i = arclength_index(s_in);
        const std::size_t i_lo = (i + 1 < samples_.size()) ? i : (samples_.size() - 2);
        const std::size_t i_hi = i_lo + 1;
        const Vector2 d = samples_[i_hi] - samples_[i_lo];
        const double n = norm(d);
        if (n <= k_vector2_epsilon) return Vector2{1.0, 0.0};
        return d / n;
    }

    Vector2 closest_point_arclength(Vector2 query, double& out_s_in) const {
        // Linear scan over discretized samples; fine for ~1000 points.
        std::size_t best_i = 0;
        double best_d2 = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < samples_.size(); ++i) {
            const Vector2 d = samples_[i] - query;
            const double  d2 = norm_sq(d);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_i  = i;
            }
        }
        // Refine onto adjacent segment for sub-sample accuracy.
        std::size_t lo = (best_i == 0) ? 0 : best_i - 1;
        std::size_t hi = std::min(best_i + 1, samples_.size() - 1);
        double best_s = cum_arclength_[best_i];
        Vector2 best_p = samples_[best_i];
        best_d2 = norm_sq(samples_[best_i] - query);
        for (std::size_t i = lo; i < hi; ++i) {
            const Vector2 a   = samples_[i];
            const Vector2 b   = samples_[i + 1];
            const Vector2 ab  = b - a;
            const double  ab2 = norm_sq(ab);
            if (ab2 <= 1e-18) continue;
            double u = dot(query - a, ab) / ab2;
            if (u < 0.0) u = 0.0;
            else if (u > 1.0) u = 1.0;
            const Vector2 p   = a + ab * u;
            const double  d2  = norm_sq(p - query);
            if (d2 < best_d2) {
                best_d2 = d2;
                best_p  = p;
                best_s  = cum_arclength_[i] + u * (cum_arclength_[i + 1] - cum_arclength_[i]);
            }
        }
        out_s_in = best_s;
        return best_p;
    }

    const std::vector<Vector2>& waypoints() const { return waypoints_; }

    // Arclength along the discretized path that the i-th original waypoint
    // landed on. Computed at construction time.
    double waypoint_arclength(std::size_t waypoint_idx) const {
        if (waypoint_idx >= waypoint_arclengths_.size()) {
            return cum_arclength_.empty() ? 0.0 : cum_arclength_.back();
        }
        return waypoint_arclengths_[waypoint_idx];
    }

    std::size_t waypoint_count() const { return waypoints_.size(); }

private:
    static constexpr double k_step_in = 0.1;

    void build_segments() {
        // Build the extended control-point list with reflected endpoints.
        const std::size_t n = waypoints_.size();
        ext_.clear();
        ext_.reserve(n + 2);
        ext_.push_back(waypoints_[0] + (waypoints_[0] - waypoints_[1]));
        for (const Vector2& w : waypoints_) ext_.push_back(w);
        ext_.push_back(waypoints_[n - 1] + (waypoints_[n - 1] - waypoints_[n - 2]));

        // Per-segment parameter spacings t_i+1 - t_i = |P_i+1 - P_i|^alpha
        // (alpha = 0.5 -> centripetal). Stored as the four parameter values
        // for each segment for direct use in the basis evaluation.
        seg_params_.clear();
        seg_params_.reserve(n - 1);
        for (std::size_t i = 0; i + 1 < n; ++i) {
            std::array<double, 4> ts{};
            ts[0] = 0.0;
            for (std::size_t k = 1; k < 4; ++k) {
                const Vector2 d = ext_[i + k] - ext_[i + k - 1];
                const double  dn = norm(d);
                ts[k] = ts[k - 1] + std::pow(std::max(dn, 1e-9), alpha_);
            }
            seg_params_.push_back(ts);
        }
    }

    Vector2 sample_segment(std::size_t i, double t) const {
        // Catmull-Rom on the four control points ext_[i..i+3] with parameter
        // values seg_params_[i]. t is in [seg_params_[i][1], seg_params_[i][2]].
        const auto& ts = seg_params_[i];
        const Vector2& p0 = ext_[i];
        const Vector2& p1 = ext_[i + 1];
        const Vector2& p2 = ext_[i + 2];
        const Vector2& p3 = ext_[i + 3];

        const double t0 = ts[0], t1 = ts[1], t2 = ts[2], t3 = ts[3];
        const Vector2 a1 = p0 * ((t1 - t) / (t1 - t0)) + p1 * ((t - t0) / (t1 - t0));
        const Vector2 a2 = p1 * ((t2 - t) / (t2 - t1)) + p2 * ((t - t1) / (t2 - t1));
        const Vector2 a3 = p2 * ((t3 - t) / (t3 - t2)) + p3 * ((t - t2) / (t3 - t2));
        const Vector2 b1 = a1 * ((t2 - t) / (t2 - t0)) + a2 * ((t - t0) / (t2 - t0));
        const Vector2 b2 = a2 * ((t3 - t) / (t3 - t1)) + a3 * ((t - t1) / (t3 - t1));
        return  b1 * ((t2 - t) / (t2 - t1)) + b2 * ((t - t1) / (t2 - t1));
    }

    void discretize() {
        samples_.clear();
        cum_arclength_.clear();
        samples_.push_back(waypoints_.front());
        cum_arclength_.push_back(0.0);

        const std::size_t n_segments = waypoints_.size() - 1;
        for (std::size_t i = 0; i < n_segments; ++i) {
            const auto& ts = seg_params_[i];
            const double t_start = ts[1];
            const double t_end   = ts[2];
            // Estimate step count from chord length; ensure at least a handful.
            const double chord = distance(waypoints_[i], waypoints_[i + 1]);
            const std::size_t steps = std::max<std::size_t>(
                4, static_cast<std::size_t>(std::ceil(chord / k_step_in)));
            for (std::size_t k = 1; k <= steps; ++k) {
                const double frac = static_cast<double>(k) / static_cast<double>(steps);
                const double t = t_start + (t_end - t_start) * frac;
                Vector2 p = sample_segment(i, t);
                if (i + 1 == n_segments && k == steps) {
                    p = waypoints_.back();
                }
                samples_.push_back(p);
                cum_arclength_.push_back(
                    cum_arclength_.back() + distance(samples_[samples_.size() - 2], p));
            }
        }
    }

    void compute_waypoint_arclengths() {
        // The discretizer pushes exactly `steps` points per segment after the
        // initial waypoint, so we can just walk the same accounting.
        waypoint_arclengths_.clear();
        waypoint_arclengths_.push_back(0.0);
        std::size_t cursor = 0;
        const std::size_t n_segments = waypoints_.size() - 1;
        for (std::size_t i = 0; i < n_segments; ++i) {
            const double chord = distance(waypoints_[i], waypoints_[i + 1]);
            const std::size_t steps = std::max<std::size_t>(
                4, static_cast<std::size_t>(std::ceil(chord / k_step_in)));
            cursor += steps;
            waypoint_arclengths_.push_back(cum_arclength_[cursor]);
        }
    }

    std::size_t arclength_index(double s_in) const {
        if (cum_arclength_.empty() || s_in <= 0.0) return 0;
        if (s_in >= cum_arclength_.back()) return cum_arclength_.size() - 1;
        // lower_bound returns first iter with value >= s_in.
        auto it = std::lower_bound(cum_arclength_.begin(), cum_arclength_.end(), s_in);
        if (it == cum_arclength_.begin()) return 0;
        --it;
        return static_cast<std::size_t>(it - cum_arclength_.begin());
    }

    std::vector<Vector2>             waypoints_;
    double                           alpha_;
    std::vector<Vector2>             ext_;
    std::vector<std::array<double, 4>> seg_params_;
    std::vector<Vector2>             samples_;
    std::vector<double>              cum_arclength_;
    std::vector<double>              waypoint_arclengths_;
};

} // namespace catmull_rom
} // namespace pathfinder
