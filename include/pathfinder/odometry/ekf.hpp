#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/matrix6.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

#include <cassert>
#include <cmath>
#include <cstddef>

namespace pathfinder {

// Tier-2 localization (spec §8 / build step 17). Extends the 6-DOF
// `DeadReckoning` state `[x, y, θ, v_x_body, v_y_body, ω]` with a full 6×6
// covariance and three measurement update kinds:
//
//   * IMU heading      — `[0, 0, 1, 0, 0, 0]` linear observation of θ.
//   * VEX GPS pose     — full `[x, y, θ]` linear observation.
//   * Landmark         — `Pose2` of a known field landmark observed in bot
//                        frame; non-linear, Jacobian computed analytically.
//
// State storage convention: matches `DeadReckoning` for pose; ω is stored in
// **rad/s internally** (per the brief), so `body_velocity()` converts to dps
// at the boundary to stay compatible with the `BodyVelocity` shape.
//
// ── Process model ────────────────────────────────────────────────────────
//
// Predict step replays the DR motion model for the *state mean* (so that
// without measurement updates, EKF.pose() ≡ DR.pose() to numerical
// precision). For the *covariance*, we propagate
//
//     P_new = F · P_old · Fᵀ + Q
//
// where F is the Jacobian of the constant-velocity differential-drive
// dynamics
//
//     x_new   = x_old + cos(θ_old + ω·dt/2)·v_x·dt − sin(θ_old + ω·dt/2)·v_y·dt
//     y_new   = y_old + sin(θ_old + ω·dt/2)·v_x·dt + cos(θ_old + ω·dt/2)·v_y·dt
//     θ_new   = θ_old + ω·dt
//     v_x_new = v_x_old                               (driven by wheel input)
//     v_y_new = (1 − μ·dt)·v_y_old − ω·v_x·dt        (cross-coupling + decay)
//     ω_new   = ω_old                                 (driven by IMU input)
//
// The non-trivial Jacobian rows (with c = cos(θ_mid), s = sin(θ_mid),
// θ_mid = θ_old + ω·dt/2) are
//
//     ∂x/∂θ = (−s·v_x − c·v_y)·dt
//     ∂x/∂vx = c·dt              ∂x/∂vy = −s·dt
//     ∂x/∂ω  = (−s·v_x − c·v_y)·dt²/2
//     ∂y/∂θ = ( c·v_x − s·v_y)·dt
//     ∂y/∂vx = s·dt              ∂y/∂vy =  c·dt
//     ∂y/∂ω  = ( c·v_x − s·v_y)·dt²/2
//     ∂θ/∂ω = dt
//     ∂vy/∂vx = −ω·dt   ∂vy/∂vy = 1 − μ·dt   ∂vy/∂ω = −v_x·dt
//
// Q is diagonal with the per-axis sigma² values from `Config`, scaled by dt
// (Brownian-motion noise growth: σ²·dt per step).
//
// ── Measurement updates ──────────────────────────────────────────────────
//
// Standard EKF formulation. For an observation z with predicted h(x), H = ∂h/∂x:
//
//     y = z − h(x)              (residual; angle components use shortest_angle)
//     S = H·P·Hᵀ + R + ε·I      (innovation covariance, ε regularizes
//                                near-singular S when sigmas are tiny)
//     K = P·Hᵀ·S⁻¹              (Kalman gain)
//     x ← x + K·y
//     P ← (I − K·H)·P           (heading components re-normalized to (−π, π])
//
// Landmark Jacobian: for a landmark at field pose `L = (Lx, Ly, Lθ)` and
// state pose `(x, y, θ)`, the predicted relative observation in bot frame is
//
//     dx_bot =  cos(θ)·(Lx − x) + sin(θ)·(Ly − y)
//     dy_bot = −sin(θ)·(Lx − x) + cos(θ)·(Ly − y)
//     dθ     = Lθ − θ                              (normalized)
//
// hence the 3×6 H matrix
//
//     H = | −cos(θ)   −sin(θ)    dy_bot   0 0 0 |
//         |  sin(θ)   −cos(θ)   −dx_bot   0 0 0 |
//         |   0          0         −1     0 0 0 |
class Ekf {
public:
    struct Config {
        // Same odometry geometry as DR (Wave A / D).
        double parallel_wheel_y_offset_in   = 0.0;
        double perp_wheel_x_offset_in       = 0.0;
        bool   has_perp_wheel               = false;
        double lateral_friction_coefficient = 5.0;   // 1/s

        // Process noise — per-axis 1-sigma per √second.
        double sigma_pos_in_per_sec      = 0.05;
        double sigma_heading_rad_per_sec = 0.005;
        double sigma_vel_ips_per_sec     = 0.10;
        double sigma_omega_rps_per_sec   = 0.05;
    };

    explicit Ekf(Config config, Pose2 initial_pose = {})
        : config_(config) {
        state_[k_x] = initial_pose.x;
        state_[k_y] = initial_pose.y;
        state_[k_theta] = initial_pose.heading.radians();
        // Velocity components default to 0; covariance defaults to 0
        // (caller seeds via process noise on the first predict() tick).
    }

    // ── Predict ──────────────────────────────────────────────────────────

    void predict(double parallel_wheel_delta_in,
                 double perp_wheel_delta_in,
                 Angle  heading_now,
                 double dt_sec) {
        assert(dt_sec > 0.0 && "Ekf::predict requires dt_sec > 0");

        if (first_predict_) {
            prev_heading_ = heading_now;
            state_[k_theta] = heading_now.radians();
            first_predict_ = false;
            return;
        }

        const double d_theta = shortest_angle(prev_heading_, heading_now).radians();

        const double d_x_body = parallel_wheel_delta_in
                              + d_theta * config_.parallel_wheel_y_offset_in;
        const double d_y_body = config_.has_perp_wheel
            ? (perp_wheel_delta_in - d_theta * config_.perp_wheel_x_offset_in)
            : 0.0;

        // 1) Linearize the constant-velocity dynamics about the *prior*
        //    state and propagate the covariance. We use the prior state's
        //    velocities (state_[k_vx]/[k_vy]/[k_omega]) as the linearization
        //    point — Q absorbs the difference between this idealization and
        //    the wheel-delta-driven mean update below.
        const Matrix6 F = motion_jacobian(dt_sec);
        const Matrix6 Q = process_noise(dt_sec);
        P_ = F * P_ * F.transpose() + Q;

        // 2) Update the state mean using the DR mid-heading integrator
        //    (replays the deterministic integration so the EKF mean lines up
        //    with DR exactly when no measurement updates have been applied).
        const double heading_mid = prev_heading_.radians() + d_theta * 0.5;
        const double c = std::cos(heading_mid);
        const double s = std::sin(heading_mid);

        state_[k_x] += c * d_x_body - s * d_y_body;
        state_[k_y] += s * d_x_body + c * d_y_body;
        state_[k_theta] = heading_now.radians();

        // 3) Update the body-velocity components from the same wheel/IMU
        //    inputs DR would use. ω stored in rad/s internally.
        const double v_x_new   = d_x_body / dt_sec;
        const double omega_new = d_theta / dt_sec;
        double       v_y_new;
        if (config_.has_perp_wheel) {
            v_y_new = d_y_body / dt_sec;
        } else {
            const double v_y_dot = -config_.lateral_friction_coefficient * state_[k_vy]
                                 - omega_new * v_x_new;
            v_y_new = state_[k_vy] + v_y_dot * dt_sec;
        }
        state_[k_vx]    = v_x_new;
        state_[k_vy]    = v_y_new;
        state_[k_omega] = omega_new;

        prev_heading_ = heading_now;
        normalize_heading();
    }

    // ── Measurement updates ──────────────────────────────────────────────

    void update_imu_heading(Angle measured_heading_bot_frame, double sigma_rad) {
        // 1×6 H = [0, 0, 1, 0, 0, 0]; reduces to scalar arithmetic.
        const double r = sigma_rad * sigma_rad + k_regularizer;
        const double residual = shortest_angle(
            Angle{state_[k_theta]}, measured_heading_bot_frame).radians();

        // S = P[2][2] + r; K = P[:,2] / S.
        const double S = P_.at(k_theta, k_theta) + r;
        Vector6 K;
        for (std::size_t i = 0; i < 6; ++i) K[i] = P_.at(i, k_theta) / S;

        for (std::size_t i = 0; i < 6; ++i) state_[i] += K[i] * residual;

        // P ← (I − K·H)·P. With H selecting only column k_theta,
        // (K·H)[i][j] = K[i] · δ(j, k_theta). So we subtract K[i]·P[k_theta][j].
        Matrix6 P_new = P_;
        for (std::size_t i = 0; i < 6; ++i)
            for (std::size_t j = 0; j < 6; ++j)
                P_new.at(i, j) -= K[i] * P_.at(k_theta, j);
        P_ = P_new;

        normalize_heading();
    }

    void update_vex_gps(Pose2 measured_pose, double sigma_xy_in, double sigma_heading_rad) {
        // H is a 3×6 of (I_3 | 0_3); the math reduces to working with the
        // top-left 3×3 of P plus the mixed pose-velocity block.
        Matrix3 R{};
        R.m[0][0] = sigma_xy_in * sigma_xy_in + k_regularizer;
        R.m[1][1] = sigma_xy_in * sigma_xy_in + k_regularizer;
        R.m[2][2] = sigma_heading_rad * sigma_heading_rad + k_regularizer;

        // Innovation covariance S = P[0..2, 0..2] + R.
        Matrix3 S{};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                S.m[i][j] = P_.at(i, j) + R.m[i][j];
        const Matrix3 S_inv = S.inverse();

        // Residual: x, y straight subtraction; heading via shortest_angle.
        Vector3 y_vec;
        y_vec.x = measured_pose.x - state_[k_x];
        y_vec.y = measured_pose.y - state_[k_y];
        y_vec.z = shortest_angle(
            Angle{state_[k_theta]}, measured_pose.heading).radians();

        // Kalman gain K = P[:,0..2] · S_inv. K is 6×3.
        double K[6][3] = {};
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 3; ++k) {
                    sum += P_.at(i, k) * S_inv.m[k][j];
                }
                K[i][j] = sum;
            }
        }

        // State update: x += K·y.
        for (std::size_t i = 0; i < 6; ++i) {
            state_[i] += K[i][0] * y_vec.x + K[i][1] * y_vec.y + K[i][2] * y_vec.z;
        }

        // Covariance update P ← (I − K·H)·P. K·H selects columns 0..2 of K
        // and zero-pads the rest, so (K·H)·P[i][j] = Σ_k K[i][k]·P[k][j],
        // k ∈ {0, 1, 2}.
        Matrix6 P_new = P_;
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 6; ++j) {
                double dec = 0.0;
                for (std::size_t k = 0; k < 3; ++k) {
                    dec += K[i][k] * P_.at(k, j);
                }
                P_new.at(i, j) -= dec;
            }
        }
        P_ = P_new;

        normalize_heading();
    }

    void update_landmark(Pose2 landmark_field_pose,
                         Pose2 relative_observation_bot_frame,
                         Matrix3 covariance) {
        // Predict the relative observation in bot frame.
        const double theta = state_[k_theta];
        const double c = std::cos(theta);
        const double s = std::sin(theta);
        const double dx_world = landmark_field_pose.x - state_[k_x];
        const double dy_world = landmark_field_pose.y - state_[k_y];

        const double dx_bot_pred =  c * dx_world + s * dy_world;
        const double dy_bot_pred = -s * dx_world + c * dy_world;
        const double dtheta_pred = shortest_angle(
            Angle{theta}, landmark_field_pose.heading).radians();

        // H = ∂h/∂x for h = (dx_bot, dy_bot, dθ).
        // Velocity columns are zero. Pose columns:
        double H[3][6] = {};
        H[0][k_x]     = -c;
        H[0][k_y]     = -s;
        H[0][k_theta] =  dy_bot_pred;
        H[1][k_x]     =  s;
        H[1][k_y]     = -c;
        H[1][k_theta] = -dx_bot_pred;
        H[2][k_theta] = -1.0;

        // Add regularizer to R.
        Matrix3 R = covariance;
        R.m[0][0] += k_regularizer;
        R.m[1][1] += k_regularizer;
        R.m[2][2] += k_regularizer;

        // Innovation covariance S = H·P·Hᵀ + R (3×3).
        // First compute H·P (3×6).
        double HP[3][6] = {};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 6; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 6; ++k) {
                    sum += H[i][k] * P_.at(k, j);
                }
                HP[i][j] = sum;
            }
        }
        Matrix3 S{};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 6; ++k) {
                    sum += HP[i][k] * H[j][k];
                }
                S.m[i][j] = sum + R.m[i][j];
            }
        }
        const Matrix3 S_inv = S.inverse();

        // Kalman gain K = P·Hᵀ·S⁻¹ (6×3). Compute P·Hᵀ first (6×3).
        double PHt[6][3] = {};
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 6; ++k) {
                    sum += P_.at(i, k) * H[j][k];
                }
                PHt[i][j] = sum;
            }
        }
        double K[6][3] = {};
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 3; ++k) {
                    sum += PHt[i][k] * S_inv.m[k][j];
                }
                K[i][j] = sum;
            }
        }

        // Residual y = z − h(x).
        Vector3 y_vec;
        y_vec.x = relative_observation_bot_frame.x - dx_bot_pred;
        y_vec.y = relative_observation_bot_frame.y - dy_bot_pred;
        y_vec.z = shortest_angle(
            Angle{dtheta_pred}, relative_observation_bot_frame.heading).radians();

        // State update.
        for (std::size_t i = 0; i < 6; ++i) {
            state_[i] += K[i][0] * y_vec.x + K[i][1] * y_vec.y + K[i][2] * y_vec.z;
        }

        // Covariance update P ← (I − K·H)·P. K·H is 6×6; compute it then
        // multiply by P. Cleaner than the structured-zero shortcut used for
        // GPS, since H here is dense in the pose columns.
        Matrix6 KH = Matrix6::zero();
        for (std::size_t i = 0; i < 6; ++i) {
            for (std::size_t j = 0; j < 6; ++j) {
                double sum = 0.0;
                for (std::size_t k = 0; k < 3; ++k) {
                    sum += K[i][k] * H[k][j];
                }
                KH.at(i, j) = sum;
            }
        }
        const Matrix6 I_minus_KH = Matrix6::identity() - KH;
        P_ = I_minus_KH * P_;

        normalize_heading();
    }

    // ── Accessors ────────────────────────────────────────────────────────

    Pose2 pose() const {
        return Pose2{state_[k_x], state_[k_y], Angle{state_[k_theta]}};
    }

    BodyVelocity body_velocity() const {
        return BodyVelocity{
            state_[k_vx],
            state_[k_vy],
            state_[k_omega] * k_rad_to_deg,
        };
    }

    Matrix3 pose_covariance() const {
        Matrix3 r{};
        for (std::size_t i = 0; i < 3; ++i)
            for (std::size_t j = 0; j < 3; ++j)
                r.m[i][j] = P_.at(i, j);
        return r;
    }

    Matrix6 full_covariance() const { return P_; }

    void set_pose(Pose2 new_pose, bool keep_velocity = false) {
        state_[k_x]     = new_pose.x;
        state_[k_y]     = new_pose.y;
        state_[k_theta] = new_pose.heading.radians();
        if (!keep_velocity) {
            state_[k_vx]    = 0.0;
            state_[k_vy]    = 0.0;
            state_[k_omega] = 0.0;
        }
        normalize_heading();
    }

    void reset() {
        prev_heading_  = Angle{};
        first_predict_ = true;
        // Velocity zeroed for the same reason DR zeroes it on reset()
        // (no Δθ baseline → no wheel-derived velocity to anchor on).
        state_[k_vx]    = 0.0;
        state_[k_vy]    = 0.0;
        state_[k_omega] = 0.0;
    }

private:
    static constexpr std::size_t k_x     = 0;
    static constexpr std::size_t k_y     = 1;
    static constexpr std::size_t k_theta = 2;
    static constexpr std::size_t k_vx    = 3;
    static constexpr std::size_t k_vy    = 4;
    static constexpr std::size_t k_omega = 5;

    // Tikhonov-style additive regularizer on innovation-covariance diagonals.
    // Prevents a singular S when sigmas are extremely small (e.g. perfect
    // GPS), at the cost of a sub-micron bias floor that's irrelevant to
    // VEX-scale precision.
    static constexpr double k_regularizer = 1e-9;

    Config  config_;
    Vector6 state_{};
    Matrix6 P_ = Matrix6::zero();
    Angle   prev_heading_{};
    bool    first_predict_ = true;

    Matrix6 motion_jacobian(double dt) const {
        const double v_x   = state_[k_vx];
        const double v_y   = state_[k_vy];
        const double omega = state_[k_omega];

        // Use the predicted mid-heading so the Jacobian linearizes about the
        // same operating point as the mean update.
        const double theta_mid = state_[k_theta] + omega * dt * 0.5;
        const double c = std::cos(theta_mid);
        const double s = std::sin(theta_mid);
        const double mu = config_.lateral_friction_coefficient;

        Matrix6 F = Matrix6::identity();
        // x row.
        F.at(k_x, k_theta) = (-s * v_x - c * v_y) * dt;
        F.at(k_x, k_vx)    =  c * dt;
        F.at(k_x, k_vy)    = -s * dt;
        F.at(k_x, k_omega) = (-s * v_x - c * v_y) * dt * dt * 0.5;
        // y row.
        F.at(k_y, k_theta) = ( c * v_x - s * v_y) * dt;
        F.at(k_y, k_vx)    =  s * dt;
        F.at(k_y, k_vy)    =  c * dt;
        F.at(k_y, k_omega) = ( c * v_x - s * v_y) * dt * dt * 0.5;
        // θ row.
        F.at(k_theta, k_omega) = dt;
        // v_x row stays identity (driven by wheel input).
        // v_y row: cross-coupling + friction decay.
        F.at(k_vy, k_vx)    = -omega * dt;
        F.at(k_vy, k_vy)    = 1.0 - mu * dt;
        F.at(k_vy, k_omega) = -v_x * dt;
        // ω row stays identity (driven by IMU input).

        return F;
    }

    Matrix6 process_noise(double dt) const {
        // Diagonal Q with σ²·dt per axis (standard random-walk discretization).
        Matrix6 Q = Matrix6::zero();
        const double q_pos     = config_.sigma_pos_in_per_sec * config_.sigma_pos_in_per_sec * dt;
        const double q_heading = config_.sigma_heading_rad_per_sec * config_.sigma_heading_rad_per_sec * dt;
        const double q_vel     = config_.sigma_vel_ips_per_sec * config_.sigma_vel_ips_per_sec * dt;
        const double q_omega   = config_.sigma_omega_rps_per_sec * config_.sigma_omega_rps_per_sec * dt;

        Q.at(k_x, k_x)         = q_pos;
        Q.at(k_y, k_y)         = q_pos;
        Q.at(k_theta, k_theta) = q_heading;
        Q.at(k_vx, k_vx)       = q_vel;
        Q.at(k_vy, k_vy)       = q_vel;
        Q.at(k_omega, k_omega) = q_omega;
        return Q;
    }

    void normalize_heading() {
        state_[k_theta] = Angle{state_[k_theta]}.normalize_signed().radians();
    }
};

} // namespace pathfinder
