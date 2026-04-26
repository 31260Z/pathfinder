#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/matrix6.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>
#include <pathfinder/odometry/ekf.hpp>

#include <cmath>
#include <cstddef>

using namespace pathfinder;

namespace {
constexpr double k_pi_local = 3.14159265358979323846;
constexpr double k_dt       = 0.01;   // 100 Hz

Ekf::Config trivial_config() {
    // Zero process noise so the predict-only-matches-DR test is exact.
    Ekf::Config cfg{};
    cfg.sigma_pos_in_per_sec      = 0.0;
    cfg.sigma_heading_rad_per_sec = 0.0;
    cfg.sigma_vel_ips_per_sec     = 0.0;
    cfg.sigma_omega_rps_per_sec   = 0.0;
    return cfg;
}
} // namespace

// ── 1. Predict-only matches DR ──────────────────────────────────────────

TEST_CASE("Ekf: predict-only mean tracks DeadReckoning identically (no perp wheel)") {
    DeadReckoning::Config dr_cfg{};
    dr_cfg.parallel_wheel_y_offset_in   = -3.0;
    dr_cfg.lateral_friction_coefficient = 1.0;
    DeadReckoning dr(dr_cfg);

    Ekf::Config ekf_cfg = trivial_config();
    ekf_cfg.parallel_wheel_y_offset_in   = -3.0;
    ekf_cfg.lateral_friction_coefficient = 1.0;
    Ekf ekf(ekf_cfg);

    dr.update(0.0, 0.0, Angle::radians(0.0), k_dt);
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);

    double heading = 0.0;
    for (int i = 0; i < 200; ++i) {
        const double d_par = 0.5;          // 50 ips
        const double d_th  = 0.005;        // ~28 dps
        heading += d_th;
        dr.update(d_par, 0.0, Angle::radians(heading), k_dt);
        ekf.predict(d_par, 0.0, Angle::radians(heading), k_dt);
    }

    const Pose2 dp = dr.pose();
    const Pose2 ep = ekf.pose();
    CHECK(ep.x == doctest::Approx(dp.x).epsilon(1e-9));
    CHECK(ep.y == doctest::Approx(dp.y).epsilon(1e-9));
    CHECK(ep.heading.radians() == doctest::Approx(dp.heading.radians()).epsilon(1e-9));

    const BodyVelocity dv = dr.body_velocity();
    const BodyVelocity ev = ekf.body_velocity();
    CHECK(ev.v_x_ips == doctest::Approx(dv.v_x_ips).epsilon(1e-9));
    CHECK(ev.v_y_ips == doctest::Approx(dv.v_y_ips).epsilon(1e-9));
    CHECK(ev.omega_dps == doctest::Approx(dv.omega_dps).epsilon(1e-9));
}

TEST_CASE("Ekf: predict-only mean tracks DR with perp wheel") {
    DeadReckoning::Config dr_cfg{};
    dr_cfg.has_perp_wheel        = true;
    dr_cfg.perp_wheel_x_offset_in = 1.5;
    DeadReckoning dr(dr_cfg);

    Ekf::Config ekf_cfg = trivial_config();
    ekf_cfg.has_perp_wheel        = true;
    ekf_cfg.perp_wheel_x_offset_in = 1.5;
    Ekf ekf(ekf_cfg);

    dr.update(0.0, 0.0, Angle::radians(0.0), k_dt);
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);

    dr.update(2.0, -0.5, Angle::radians(0.1), k_dt);
    ekf.predict(2.0, -0.5, Angle::radians(0.1), k_dt);

    const Pose2 dp = dr.pose();
    const Pose2 ep = ekf.pose();
    CHECK(ep.x == doctest::Approx(dp.x).epsilon(1e-12));
    CHECK(ep.y == doctest::Approx(dp.y).epsilon(1e-12));
    CHECK(ep.heading.radians() == doctest::Approx(dp.heading.radians()).epsilon(1e-12));
}

// ── 2. IMU update reduces heading uncertainty ───────────────────────────

TEST_CASE("Ekf: IMU heading update shrinks heading variance") {
    Ekf ekf(trivial_config(), Pose2{0.0, 0.0, Angle::radians(0.0)});

    // Seed a large prior θ uncertainty by hand.
    Matrix6 P = ekf.full_covariance();
    P.at(2, 2) = 0.5 * 0.5;   // σ = 0.5 rad
    // Hack: there's no setter for P, so we drive the covariance up via predict
    // ticks with non-zero process noise instead.
    Ekf::Config cfg = trivial_config();
    cfg.sigma_heading_rad_per_sec = 1.0;   // huge process noise
    Ekf ekf2(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});
    ekf2.predict(0.0, 0.0, Angle::radians(0.0), k_dt);   // seed
    for (int i = 0; i < 10; ++i) {
        ekf2.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    }
    const double sigma_before = std::sqrt(ekf2.pose_covariance().m[2][2]);
    REQUIRE(sigma_before > 0.1);

    ekf2.update_imu_heading(Angle::radians(0.0), 0.01);   // very confident IMU

    const double sigma_after = std::sqrt(ekf2.pose_covariance().m[2][2]);
    CHECK(sigma_after < sigma_before);
    // After a tight measurement, the posterior σ should be near the
    // measurement σ (Kalman fusion of two Gaussians).
    CHECK(sigma_after < 0.02);
}

// ── 3. VEX GPS update locks position ─────────────────────────────────────

TEST_CASE("Ekf: VexGps update snaps state to measurement and shrinks covariance") {
    Ekf::Config cfg = trivial_config();
    cfg.sigma_pos_in_per_sec      = 1.0;   // 1 in/√s
    cfg.sigma_heading_rad_per_sec = 0.1;
    Ekf ekf(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});

    // Drive up the position and heading covariance by simulating a long
    // dead-reckoning span with large process noise.
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 100; ++i) {
        ekf.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    }
    const double xx_before    = ekf.pose_covariance().m[0][0];
    const double yy_before    = ekf.pose_covariance().m[1][1];
    const double thetheta_before = ekf.pose_covariance().m[2][2];
    REQUIRE(xx_before > 1.0);
    REQUIRE(yy_before > 1.0);

    // Tight GPS observation at (10, 5, π/4).
    ekf.update_vex_gps(Pose2{10.0, 5.0, Angle::radians(k_pi_local / 4.0)},
                       /*sigma_xy_in*/ 0.01, /*sigma_heading_rad*/ 0.005);

    const Pose2 p = ekf.pose();
    CHECK(p.x == doctest::Approx(10.0).epsilon(0.001));
    CHECK(p.y == doctest::Approx(5.0).epsilon(0.001));
    CHECK(p.heading.radians() == doctest::Approx(k_pi_local / 4.0).epsilon(0.001));

    const Matrix3 cov = ekf.pose_covariance();
    CHECK(cov.m[0][0] < xx_before);
    CHECK(cov.m[1][1] < yy_before);
    CHECK(cov.m[2][2] < thetheta_before);
    // Posterior σ should be near the measurement σ.
    CHECK(std::sqrt(cov.m[0][0]) < 0.02);
    CHECK(std::sqrt(cov.m[1][1]) < 0.02);
}

// ── 4. Landmark update with consistent / inconsistent observation ───────

TEST_CASE("Ekf: landmark observation matching the prediction leaves state put") {
    Ekf ekf(trivial_config(), Pose2{0.0, 0.0, Angle::radians(0.0)});
    // Drive up some uncertainty so the gain is non-zero.
    Ekf::Config cfg = trivial_config();
    cfg.sigma_pos_in_per_sec = 0.1;
    Ekf ekf2(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});
    ekf2.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 10; ++i) {
        ekf2.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    }

    const Pose2 landmark{10.0, 0.0, Angle::radians(0.0)};
    // Bot at origin facing +X → landmark observed in bot frame at (10, 0, 0).
    Matrix3 R = Matrix3::identity() * (0.05 * 0.05);
    ekf2.update_landmark(landmark,
                         Pose2{10.0, 0.0, Angle::radians(0.0)},
                         R);

    const Pose2 p = ekf2.pose();
    CHECK(p.x == doctest::Approx(0.0).epsilon(0.001));
    CHECK(p.y == doctest::Approx(0.0).epsilon(0.001));
    CHECK(p.heading.radians() == doctest::Approx(0.0).epsilon(0.001));
}

TEST_CASE("Ekf: landmark observation corrects perturbed pose") {
    // True pose: at origin facing +X. Landmark at (10, 0, 0).
    // Suppose the EKF *thinks* the bot is at (1, 0, 0) (1 inch behind truth),
    // but the camera observes the landmark at relative bot-frame (10, 0, 0)
    // — i.e. the observation is consistent with the true pose, not the
    // estimate. The EKF should pull the estimate back toward (0, 0, 0).
    Ekf::Config cfg = trivial_config();
    cfg.sigma_pos_in_per_sec      = 1.0;
    cfg.sigma_heading_rad_per_sec = 0.1;
    Ekf ekf(cfg, Pose2{1.0, 0.0, Angle::radians(0.0)});
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 50; ++i) {
        ekf.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    }

    const Pose2 landmark{10.0, 0.0, Angle::radians(0.0)};
    Matrix3 R = Matrix3::identity() * (0.05 * 0.05);
    ekf.update_landmark(landmark,
                        Pose2{10.0, 0.0, Angle::radians(0.0)},
                        R);

    const Pose2 p = ekf.pose();
    // The state should have moved noticeably from (1, 0, 0) toward (0, 0, 0).
    CHECK(p.x < 0.5);                 // moved at least halfway
    CHECK(p.x == doctest::Approx(0.0).epsilon(0.1));
    CHECK(std::abs(p.y) < 0.2);
}

TEST_CASE("Ekf: landmark observation off-axis corrects heading and y") {
    // Bot believes it's at (0, 0, 0), but actually the landmark seen at
    // bot-frame (10, 0, 0) is at field (10, 1, 0) — implies the bot is
    // displaced by (0, -1, 0) from where it thinks.
    Ekf::Config cfg = trivial_config();
    cfg.sigma_pos_in_per_sec      = 1.0;
    cfg.sigma_heading_rad_per_sec = 0.05;
    Ekf ekf(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 50; ++i) {
        ekf.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    }

    const Pose2 landmark{10.0, 1.0, Angle::radians(0.0)};
    Matrix3 R = Matrix3::identity() * (0.05 * 0.05);
    ekf.update_landmark(landmark,
                        Pose2{10.0, 0.0, Angle::radians(0.0)},
                        R);

    const Pose2 p = ekf.pose();
    // Bot's y should shift toward +1 (since landmark is at field y=1 and
    // the observation says it's directly ahead).
    CHECK(p.y > 0.5);
    CHECK(p.y == doctest::Approx(1.0).epsilon(0.1));
}

// ── 5. Heading wraparound ───────────────────────────────────────────────

TEST_CASE("Ekf: IMU update across heading wrap uses shortest_angle residual") {
    // Estimate at θ = π − 0.05; measurement at θ = −π + 0.05. The physical
    // difference is +0.10 rad (or equivalently +0.10 mod 2π); naive
    // subtraction would give ≈ −2π+0.10. Verify the state moves the small
    // way around the circle, not the large way.
    Ekf::Config cfg = trivial_config();
    cfg.sigma_heading_rad_per_sec = 1.0;
    Ekf ekf(cfg, Pose2{0.0, 0.0, Angle::radians(k_pi_local - 0.05)});
    ekf.predict(0.0, 0.0, Angle::radians(k_pi_local - 0.05), k_dt);
    // Build up some θ uncertainty.
    for (int i = 0; i < 50; ++i) {
        ekf.predict(0.0, 0.0, Angle::radians(k_pi_local - 0.05), 0.1);
    }

    ekf.update_imu_heading(Angle::radians(-k_pi_local + 0.05), 0.01);

    // Posterior heading should be close to ±π (it's the same physical
    // direction); normalized to (−π, π], the small residual means the state
    // moves to about −π+0.05 (the measurement, since we trust it heavily).
    const double th = ekf.pose().heading.radians();
    // Two acceptable outcomes (depending on whether wrap puts the result on
    // the +π or −π side): both are physically the "behind" direction.
    const bool near_neg = std::abs(th - (-k_pi_local + 0.05)) < 0.05;
    const bool near_pos = std::abs(th - (k_pi_local - 0.05) - 0.10) < 0.05;
    CHECK((near_neg || near_pos));
    // It must NOT have flipped to the opposite hemisphere by ~2π distance.
    CHECK(std::abs(th) > k_pi_local - 0.5);
}

// ── 6. Process-noise injection grows P with no measurements ─────────────

TEST_CASE("Ekf: covariance grows under process noise when no measurements arrive") {
    Ekf::Config cfg{};
    cfg.sigma_pos_in_per_sec      = 0.1;
    cfg.sigma_heading_rad_per_sec = 0.05;
    cfg.sigma_vel_ips_per_sec     = 0.2;
    cfg.sigma_omega_rps_per_sec   = 0.1;
    Ekf ekf(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});

    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    const Matrix6 P0 = ekf.full_covariance();
    // Right after the seeding (first) tick, the per-axis variances are the
    // process-noise injections from one Q step (Q = σ²·dt).
    // Sanity: x diagonal should equal σ_pos² · dt = 0.01·0.01 = 1e-4.
    // Actually first_predict_ skips Q, so let's run a few ticks first.
    for (int i = 0; i < 100; ++i) ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    const Matrix6 P1 = ekf.full_covariance();

    // After 100 more ticks of dt=0.01 (1 s total), the diagonal should grow
    // monotonically (since Q ≥ 0 and F doesn't shrink).
    CHECK(P1.at(0, 0) > P0.at(0, 0));
    CHECK(P1.at(1, 1) > P0.at(1, 1));
    CHECK(P1.at(2, 2) > P0.at(2, 2));
    CHECK(P1.at(3, 3) > P0.at(3, 3));
    CHECK(P1.at(5, 5) > P0.at(5, 5));

    // Order-of-magnitude check: x variance after T=1 s with σ=0.1 should be
    // around σ²·T = 0.01 (plus contributions from coupled velocity terms).
    const double sigma_x = std::sqrt(P1.at(0, 0));
    CHECK(sigma_x > 0.05);
    CHECK(sigma_x < 1.0);
}

// ── Accessors and lifecycle ─────────────────────────────────────────────

TEST_CASE("Ekf: default state is at the origin facing +X, zero velocity") {
    Ekf ekf({});
    const Pose2 p = ekf.pose();
    CHECK(p.x == doctest::Approx(0.0));
    CHECK(p.y == doctest::Approx(0.0));
    CHECK(p.heading.radians() == doctest::Approx(0.0));
    const BodyVelocity v = ekf.body_velocity();
    CHECK(v.v_x_ips == doctest::Approx(0.0));
    CHECK(v.v_y_ips == doctest::Approx(0.0));
    CHECK(v.omega_dps == doctest::Approx(0.0));
}

TEST_CASE("Ekf: initial pose is preserved before any predicts") {
    Pose2 start{5.0, -2.0, Angle::degrees(30.0)};
    Ekf ekf({}, start);
    CHECK(ekf.pose() == start);
}

TEST_CASE("Ekf: set_pose teleports and (by default) zeros velocity") {
    Ekf::Config cfg = trivial_config();
    cfg.has_perp_wheel = true;
    Ekf ekf(cfg);
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    ekf.predict(0.5, 0.3, Angle::radians(0.0), k_dt);
    REQUIRE(ekf.body_velocity().v_x_ips != doctest::Approx(0.0));

    ekf.set_pose(Pose2{1.0, 2.0, Angle{}});
    const BodyVelocity v = ekf.body_velocity();
    CHECK(v.v_x_ips == doctest::Approx(0.0));
    CHECK(v.v_y_ips == doctest::Approx(0.0));
    CHECK(v.omega_dps == doctest::Approx(0.0));
    CHECK(ekf.pose() == Pose2{1.0, 2.0, Angle{}});
}

TEST_CASE("Ekf: set_pose with keep_velocity=true preserves velocity") {
    Ekf::Config cfg = trivial_config();
    cfg.has_perp_wheel = true;
    Ekf ekf(cfg);
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    ekf.predict(0.5, 0.3, Angle::radians(0.0), k_dt);
    const BodyVelocity v_before = ekf.body_velocity();

    ekf.set_pose(Pose2{1.0, 2.0, Angle{}}, /*keep_velocity*/ true);
    const BodyVelocity v_after = ekf.body_velocity();
    CHECK(v_after.v_x_ips == doctest::Approx(v_before.v_x_ips));
    CHECK(v_after.v_y_ips == doctest::Approx(v_before.v_y_ips));
}

TEST_CASE("Ekf: reset clears integration history; next predict is the seed tick") {
    Ekf ekf(trivial_config(), Pose2{3.0, 4.0, Angle::radians(0.5)});
    ekf.predict(0.0, 0.0, Angle::radians(0.5), k_dt);
    ekf.predict(5.0, 0.0, Angle::radians(0.5), k_dt);
    const Pose2 before = ekf.pose();

    ekf.reset();
    CHECK(ekf.pose() == before);   // pose unchanged

    // First post-reset predict is a seed; nonzero wheel reading must NOT move
    // the pose.
    ekf.predict(99.0, 0.0, Angle::radians(0.5), k_dt);
    CHECK(ekf.pose() == before);
}

TEST_CASE("Ekf: pose_covariance returns the top-left 3x3 of the full covariance") {
    Ekf::Config cfg{};
    cfg.sigma_pos_in_per_sec = 0.5;
    Ekf ekf(cfg);
    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 50; ++i) ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);

    const Matrix3 sub  = ekf.pose_covariance();
    const Matrix6 full = ekf.full_covariance();
    for (std::size_t i = 0; i < 3; ++i)
        for (std::size_t j = 0; j < 3; ++j)
            CHECK(sub.m[i][j] == doctest::Approx(full.at(i, j)));
}

TEST_CASE("Ekf: set_pose normalizes heading into (-pi, pi]") {
    Ekf ekf({});
    ekf.set_pose(Pose2{0.0, 0.0, Angle::radians(3.0 * k_pi_local)});
    // 3π normalizes to π (or -π depending on sign convention; either is fine
    // physically — verify it's not 3π).
    const double th = ekf.pose().heading.radians();
    CHECK(std::abs(th) <= k_pi_local + 1e-9);
}

// ── IMU update sanity ───────────────────────────────────────────────────

TEST_CASE("Ekf: IMU update with finite sigma pulls heading partway") {
    Ekf::Config cfg = trivial_config();
    cfg.sigma_heading_rad_per_sec = 0.1;
    Ekf ekf(cfg, Pose2{0.0, 0.0, Angle::radians(0.0)});

    ekf.predict(0.0, 0.0, Angle::radians(0.0), k_dt);
    for (int i = 0; i < 25; ++i) ekf.predict(0.0, 0.0, Angle::radians(0.0), 0.1);
    // Now σ_θ ≈ 0.1·√(2.5) ≈ 0.158 rad. Apply a measurement at 0.3 rad with
    // measurement σ matching the prior σ → posterior should split halfway.
    const double sigma_prior = std::sqrt(ekf.pose_covariance().m[2][2]);
    ekf.update_imu_heading(Angle::radians(0.3), sigma_prior);
    const double th = ekf.pose().heading.radians();
    CHECK(th > 0.05);
    CHECK(th < 0.25);
}
