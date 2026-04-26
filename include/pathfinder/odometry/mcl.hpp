#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/odometry/field_map.hpp>
#include <pathfinder/sensors/distance_sensor.hpp>
#include <pathfinder/sensors/landmark_sensor_config.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace pathfinder {

// Tier-3 localization: a particle filter (Monte Carlo Localization) against
// the static field map. Maintains a population of weighted Pose2 hypotheses,
// each advanced by the same body-frame motion delta + Gaussian process noise,
// and re-weighted on each sensor update by the sensor likelihood.
//
// Spec §8 ("Tier 3 — MCL"): 200–500 particles; predict-update-resample loop;
// low-variance / systematic resampling triggered when ESS = 1/Σwᵢ² drops
// below threshold; weighted-mean pose with circular-mean heading.
//
// Sensor models:
//   • Distance sensor — for each particle, ray-cast from the sensor's world
//     position in the sensor's world heading; likelihood is a Gaussian on
//     (measured − expected). Skips particles inside an obstacle (their
//     ray-cast returns 0; treated as a near-impossible reading and weight-
//     ed accordingly).
//   • Landmark sensor — observes the relative pose of a known landmark.
//     For each particle, predict the relative observation as
//     `landmark_field_pose · particle.inverse()` (the landmark expressed in
//     the particle's body frame), and likelihood-weight the residual using
//     diagonal sigmas on (Δx, Δy, Δheading).
//
// The RNG is std::mt19937_64, seeded from the config. Deterministic.
class Mcl {
public:
    struct Config {
        FieldMap field_map = FieldMap::default_perimeter();
        std::size_t particle_count            = 300;

        double process_sigma_xy_in            = 0.05;
        double process_sigma_heading_rad      = 0.005;

        double distance_sigma_in              = 0.5;
        double landmark_sigma_xy_in           = 0.5;
        double landmark_sigma_heading_rad     = 0.05;

        double ess_resample_threshold         = 0.5;

        std::uint64_t rng_seed                = 0xC0FFEEULL;
    };

    struct Particle {
        Pose2  pose;
        double weight;
    };

    explicit Mcl(Config config,
                 Pose2  initial_pose             = {},
                 double initial_xy_sigma_in      = 1.0,
                 double initial_heading_sigma_rad = 0.1)
        : config_(std::move(config)),
          rng_(config_.rng_seed) {
        if (config_.particle_count == 0) {
            config_.particle_count = 1;   // Pathological-but-valid degenerate case.
        }
        seed_particles(initial_pose, initial_xy_sigma_in, initial_heading_sigma_rad);
    }

    // ── Predict ──────────────────────────────────────────────────────────
    // (forward, lateral, heading_delta) is the body-frame motion delta from
    // the previous step (DR / EKF output for one cycle). Each particle
    // applies this delta in its own body frame and adds Gaussian process
    // noise per `process_sigma_*`.
    void predict(double forward_in_bot_frame,
                 double lateral_in_bot_frame,
                 double heading_delta_rad) {
        std::normal_distribution<double> n_xy(0.0, config_.process_sigma_xy_in);
        std::normal_distribution<double> n_h (0.0, config_.process_sigma_heading_rad);
        for (Particle& p : particles_) {
            // Transform the body-frame delta into the particle's world frame
            // using the particle's *current* heading (forward-Euler).
            const double c = std::cos(p.pose.heading.rad);
            const double s = std::sin(p.pose.heading.rad);
            const double dx_world = c * forward_in_bot_frame - s * lateral_in_bot_frame;
            const double dy_world = s * forward_in_bot_frame + c * lateral_in_bot_frame;

            p.pose.x       += dx_world + n_xy(rng_);
            p.pose.y       += dy_world + n_xy(rng_);
            p.pose.heading  = (p.pose.heading + Angle{heading_delta_rad + n_h(rng_)}).normalize_signed();
        }
    }

    // ── Update from distance sensors ─────────────────────────────────────
    void update_distance(const std::vector<DistanceSensor>& sensors) {
        if (sensors.empty()) return;
        // Cache the (measured, σ, max_range, sensor offset, sensor heading)
        // tuples once before the per-particle loop.
        struct Reading {
            double  measured;
            double  sigma;
            double  max_range;
            Vector2 offset_xy;
            double  theta_rad;
            bool    valid;
        };
        std::vector<Reading> readings;
        readings.reserve(sensors.size());
        for (const DistanceSensor& s : sensors) {
            Reading r{};
            r.offset_xy = s.offset_xy_in;
            r.theta_rad = s.theta_deg * k_deg_to_rad;
            r.max_range = s.max_range_in;
            r.sigma     = std::max(s.sigma_in, 1e-6);
            if (s.sensor && s.sensor->is_valid()) {
                r.measured = s.sensor->distance_in();
                r.valid    = true;
            } else {
                r.measured = s.max_range_in + 1.0;   // out of range
                r.valid    = false;
            }
            readings.push_back(r);
        }

        for (Particle& p : particles_) {
            double log_w = 0.0;
            for (const Reading& r : readings) {
                if (!r.valid) continue;
                const Vector2 sensor_world = p.pose.transform(r.offset_xy);
                const double  beam_dir     = p.pose.heading.rad + r.theta_rad;
                const double  expected     = config_.field_map.cast_ray(
                    sensor_world, beam_dir, r.max_range);
                const double  diff         = r.measured - expected;
                log_w -= 0.5 * (diff * diff) / (r.sigma * r.sigma);
            }
            p.weight *= std::exp(log_w);
        }

        normalize_weights();
        maybe_resample();
    }

    // ── Update from landmark observations ────────────────────────────────
    void update_landmarks(const std::vector<LandmarkSensorConfig>& sensors) {
        if (sensors.empty()) return;
        struct Obs {
            LandmarkObservation  obs;
            Vector2              sensor_offset_xy;
        };
        std::vector<Obs> obs_list;
        for (const LandmarkSensorConfig& s : sensors) {
            if (!s.sensor) continue;
            auto maybe = s.sensor->poll();
            if (!maybe) continue;
            obs_list.push_back({*maybe, s.offset_xy_in});
        }
        if (obs_list.empty()) return;

        const double sigma_xy   = std::max(config_.landmark_sigma_xy_in,        1e-6);
        const double sigma_th   = std::max(config_.landmark_sigma_heading_rad,  1e-6);
        const double inv_var_xy = 1.0 / (sigma_xy * sigma_xy);
        const double inv_var_th = 1.0 / (sigma_th * sigma_th);

        for (Particle& p : particles_) {
            double log_w = 0.0;
            for (const Obs& o : obs_list) {
                // Predict the relative observation: where would this landmark
                // appear, expressed in the particle's body frame?
                //   relative_predicted = particle.inverse() · landmark_field_pose
                const Pose2 predicted = p.pose.inverse().compose(o.obs.landmark_field_pose);
                const double dx = o.obs.relative_observation.x       - predicted.x;
                const double dy = o.obs.relative_observation.y       - predicted.y;
                const double dh = shortest_angle(predicted.heading,
                                                 o.obs.relative_observation.heading).rad;
                log_w -= 0.5 * ((dx * dx + dy * dy) * inv_var_xy + dh * dh * inv_var_th);
            }
            p.weight *= std::exp(log_w);
        }

        normalize_weights();
        maybe_resample();
    }

    // ── Estimate ─────────────────────────────────────────────────────────
    Pose2 pose() const {
        double sum_x   = 0.0;
        double sum_y   = 0.0;
        double sum_sin = 0.0;
        double sum_cos = 0.0;
        double sum_w   = 0.0;
        for (const Particle& p : particles_) {
            sum_x   += p.weight * p.pose.x;
            sum_y   += p.weight * p.pose.y;
            sum_sin += p.weight * std::sin(p.pose.heading.rad);
            sum_cos += p.weight * std::cos(p.pose.heading.rad);
            sum_w   += p.weight;
        }
        if (sum_w <= 0.0) {
            // Degenerate: every particle has zero weight (pathological sensor
            // disagreement). Return the unweighted mean as a fallback.
            return unweighted_mean();
        }
        return Pose2{
            sum_x / sum_w,
            sum_y / sum_w,
            Angle{std::atan2(sum_sin, sum_cos)},
        };
    }

    // Sample (population) covariance of the particle cloud, using the
    // weighted mean as the centroid. Heading variance is computed in tangent
    // space (residual from the circular mean). All cross-terms are populated.
    Matrix3 pose_covariance() const {
        const Pose2 mean = pose();
        const double mean_h = mean.heading.rad;
        double sum_w = 0.0;
        for (const Particle& p : particles_) sum_w += p.weight;
        if (sum_w <= 0.0) return Matrix3::zero();

        double sxx = 0.0, sxy = 0.0, sxh = 0.0;
        double syy = 0.0, syh = 0.0;
        double shh = 0.0;
        for (const Particle& p : particles_) {
            const double dx = p.pose.x - mean.x;
            const double dy = p.pose.y - mean.y;
            const double dh = shortest_angle(Angle{mean_h}, p.pose.heading).rad;
            sxx += p.weight * dx * dx;
            sxy += p.weight * dx * dy;
            sxh += p.weight * dx * dh;
            syy += p.weight * dy * dy;
            syh += p.weight * dy * dh;
            shh += p.weight * dh * dh;
        }
        Matrix3 cov{};
        cov.m[0][0] = sxx / sum_w; cov.m[0][1] = sxy / sum_w; cov.m[0][2] = sxh / sum_w;
        cov.m[1][0] = sxy / sum_w; cov.m[1][1] = syy / sum_w; cov.m[1][2] = syh / sum_w;
        cov.m[2][0] = sxh / sum_w; cov.m[2][1] = syh / sum_w; cov.m[2][2] = shh / sum_w;
        return cov;
    }

    const std::vector<Particle>& particles() const { return particles_; }

    void set_pose(Pose2 new_pose,
                  double xy_sigma_in       = 1.0,
                  double heading_sigma_rad = 0.1) {
        seed_particles(new_pose, xy_sigma_in, heading_sigma_rad);
    }

    // Reset back to the initial-pose distribution at the origin. Mostly used
    // by tests; production callers prefer set_pose with the actual start pose.
    void reset() {
        seed_particles(Pose2{}, 1.0, 0.1);
    }

private:
    // ── Helpers ──────────────────────────────────────────────────────────
    void seed_particles(Pose2 mean, double xy_sigma, double h_sigma) {
        particles_.clear();
        particles_.reserve(config_.particle_count);
        const double w0 = 1.0 / static_cast<double>(config_.particle_count);
        std::normal_distribution<double> n_xy(0.0, std::max(xy_sigma, 1e-6));
        std::normal_distribution<double> n_h (0.0, std::max(h_sigma,  1e-6));
        for (std::size_t i = 0; i < config_.particle_count; ++i) {
            Pose2 p = mean;
            p.x       += n_xy(rng_);
            p.y       += n_xy(rng_);
            p.heading  = (p.heading + Angle{n_h(rng_)}).normalize_signed();
            particles_.push_back({p, w0});
        }
    }

    Pose2 unweighted_mean() const {
        if (particles_.empty()) return {};
        double sx = 0, sy = 0, ss = 0, sc = 0;
        for (const Particle& p : particles_) {
            sx += p.pose.x;
            sy += p.pose.y;
            ss += std::sin(p.pose.heading.rad);
            sc += std::cos(p.pose.heading.rad);
        }
        const double n = static_cast<double>(particles_.size());
        return Pose2{sx / n, sy / n, Angle{std::atan2(ss, sc)}};
    }

    void normalize_weights() {
        double sum = 0.0;
        for (const Particle& p : particles_) sum += p.weight;
        if (sum <= 0.0) {
            // All weights collapsed (sensor model said "impossible" for every
            // particle). Restart with uniform weights so the next update has
            // something to work with — a soft "kidnapped robot" recovery.
            const double w0 = 1.0 / static_cast<double>(particles_.size());
            for (Particle& p : particles_) p.weight = w0;
            return;
        }
        const double inv = 1.0 / sum;
        for (Particle& p : particles_) p.weight *= inv;
    }

    double effective_sample_size() const {
        double s2 = 0.0;
        for (const Particle& p : particles_) s2 += p.weight * p.weight;
        if (s2 <= 0.0) return static_cast<double>(particles_.size());
        return 1.0 / s2;
    }

    void maybe_resample() {
        const double ess = effective_sample_size();
        const double thr = config_.ess_resample_threshold *
                           static_cast<double>(particles_.size());
        if (ess >= thr) return;
        systematic_resample();
    }

    // Low-variance / systematic resampling (Thrun et al., Probabilistic
    // Robotics §4.3.4). Draws N particles in a single sweep using a single
    // uniform random offset — much lower-variance than independent draws.
    void systematic_resample() {
        const std::size_t n = particles_.size();
        std::uniform_real_distribution<double> u(0.0, 1.0 / static_cast<double>(n));
        const double r = u(rng_);
        std::vector<Particle> out;
        out.reserve(n);

        double c = particles_[0].weight;
        std::size_t i = 0;
        for (std::size_t m = 0; m < n; ++m) {
            const double u_m = r + static_cast<double>(m) / static_cast<double>(n);
            while (u_m > c && i + 1 < n) {
                ++i;
                c += particles_[i].weight;
            }
            Particle p = particles_[i];
            p.weight   = 1.0 / static_cast<double>(n);
            out.push_back(p);
        }
        particles_ = std::move(out);
    }

    Config                config_;
    std::mt19937_64       rng_;
    std::vector<Particle> particles_;
};

} // namespace pathfinder
