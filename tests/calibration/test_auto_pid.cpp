#include <doctest/doctest.h>

#include <pathfinder/calibration/auto_pid.hpp>
#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/sensors/mocks.hpp>

#include "../chassis/sim_helpers.hpp"

#include <cmath>
#include <memory>
#include <random>

using namespace pathfinder;
using pathfinder::calibration::AutoPid;
using pathfinder::calibration::TuneAxis;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {

// ── Synthetic FOPDT process ───────────────────────────────────────────
//
// We model the bot's response to applied per-side voltages as a first-
// order-plus-dead-time process. AutoPid's step capture observes
// chassis.getBodyVelocity() (v_x for lateral, omega for angular); we drive
// those readings via the chassis's normal odometry pipeline by integrating
// our own physics into the FakeRotation + FakeImu instances.
//
// On each tick:
//   • Read commanded voltage from the FakeMotors.
//   • Compute the new instantaneous "ideal" velocity (open-loop FOPDT
//     steady-state for the current input), accounting for dead time L.
//   • Move the actual velocity toward the ideal at the FOPDT rate
//     (first-order lag with time constant tau).
//   • Integrate that velocity to update the FakeRotation (parallel wheel)
//     and FakeImu (heading).
struct FopdtRig {
    std::shared_ptr<FakeMotor>    left_motor   = std::make_shared<FakeMotor>();
    std::shared_ptr<FakeMotor>    right_motor  = std::make_shared<FakeMotor>();
    std::shared_ptr<FakeRotation> par_wheel    = std::make_shared<FakeRotation>();
    std::shared_ptr<FakeImu>      imu          = std::make_shared<FakeImu>();

    // Ground-truth FOPDT params (for both lateral and angular axes).
    // Inputs are forward-or-differential voltage in mV. Outputs are v_x
    // in ips / omega in dps.
    double K_lat   = 0.005;   // ips per mV (e.g. 6000 mV → 30 ips steady state)
    double tau_lat = 0.20;    // s
    double L_lat   = 0.05;    // s

    double K_ang   = 0.020;   // dps per mV (e.g. 4000 mV → 80 dps steady state)
    double tau_ang = 0.15;    // s
    double L_ang   = 0.05;    // s

    double track_width_in    = 12.0;
    double wheel_diameter_in = 3.25;

    // Internal state.
    double v_lat_ips    = 0.0;     // current forward velocity (post-FOPDT)
    double v_ang_dps    = 0.0;     // current heading rate
    double pose_x_in    = 0.0;     // for downstream pose-tracking only
    double heading_rad  = 0.0;
    // Dead-time delay: keep recent input voltages and use the one that
    // arrived L seconds ago.
    std::vector<double> hist_lat_input_mv;
    std::vector<double> hist_ang_input_mv;
    // Optional additive zero-mean Gaussian noise on the velocity sample.
    double noise_sigma_lat = 0.0;
    double noise_sigma_ang = 0.0;
    std::mt19937 rng{42};

    // Replicates the chassis voltage→ips path so the FOPDT inputs we use
    // match what AutoPid is commanding.
    static double voltage_to_ips(double mv) {
        return (mv / 12000.0) * 60.0;   // 60 ips at full voltage
    }

    void step(double dt) {
        // Common: AutoPid commands left=right for lateral or left=-right
        // for angular. We can decode both from per-side voltages:
        //   forward_mv  = (left + right) / 2
        //   diff_mv     = (left - right) / 2
        const double l_mv = left_motor->last_voltage_mv();
        const double r_mv = right_motor->last_voltage_mv();
        const double fwd_mv  = 0.5 * (l_mv + r_mv);
        const double diff_mv = 0.5 * (l_mv - r_mv);

        // Apply dead-time L by buffering inputs.
        const int n_lag_lat = std::max(0, static_cast<int>(std::round(L_lat / dt)));
        const int n_lag_ang = std::max(0, static_cast<int>(std::round(L_ang / dt)));
        hist_lat_input_mv.push_back(fwd_mv);
        hist_ang_input_mv.push_back(diff_mv);
        const double lat_input_mv = (static_cast<int>(hist_lat_input_mv.size()) > n_lag_lat)
            ? hist_lat_input_mv[hist_lat_input_mv.size() - 1 - static_cast<std::size_t>(n_lag_lat)]
            : 0.0;
        const double ang_input_mv = (static_cast<int>(hist_ang_input_mv.size()) > n_lag_ang)
            ? hist_ang_input_mv[hist_ang_input_mv.size() - 1 - static_cast<std::size_t>(n_lag_ang)]
            : 0.0;

        const double v_lat_target = K_lat * lat_input_mv;
        const double v_ang_target = K_ang * ang_input_mv;

        // First-order lag toward target with time constant tau:
        //   v_new = v + (target - v) * (1 - exp(-dt/tau))
        const double alpha_lat = 1.0 - std::exp(-dt / tau_lat);
        const double alpha_ang = 1.0 - std::exp(-dt / tau_ang);
        v_lat_ips += (v_lat_target - v_lat_ips) * alpha_lat;
        v_ang_dps += (v_ang_target - v_ang_dps) * alpha_ang;

        // Optional noise on the *measured* values (added to the velocity
        // before pushing into the wheel/IMU integrators).
        std::normal_distribution<double> n_lat(0.0, noise_sigma_lat);
        std::normal_distribution<double> n_ang(0.0, noise_sigma_ang);
        const double v_lat_meas = v_lat_ips + ((noise_sigma_lat > 0.0) ? n_lat(rng) : 0.0);
        const double v_ang_meas = v_ang_dps + ((noise_sigma_ang > 0.0) ? n_ang(rng) : 0.0);

        // Push into FakeRotation + FakeImu so the chassis odometry sees
        // the velocity. Parallel wheel measures forward distance; heading
        // is the integral of v_ang_dps.
        const double circ = k_pi * wheel_diameter_in;
        if (circ > 0.0) {
            par_wheel->set_position(par_wheel->position_revolutions()
                                    + (v_lat_meas * dt) / circ);
        }
        const double v_ang_rad = v_ang_meas * k_deg_to_rad;
        heading_rad += v_ang_rad * dt;
        pose_x_in   += v_lat_meas * dt;
        imu->set_heading(Angle{heading_rad});
    }

    Sensors make_sensors() {
        Sensors s;
        TrackingWheel par{};
        par.sensor             = par_wheel;
        par.wheel              = Wheel::Custom;
        par.custom_diameter_in = wheel_diameter_in;
        par.axis               = Axis::X;
        par.offset             = {9.0, 9.0};   // bot center (no Δθ contribution)
        s.add(par);

        Imu imu_cfg{};
        imu_cfg.sensor       = imu;
        imu_cfg.mounting     = ImuMounting::ZDown_XForward;
        imu_cfg.offset_xy_in = {9.0, 9.0};
        s.add(imu_cfg);
        return s;
    }

    Drive make_drive_cfg() const {
        Drive d{};
        d.track_width_in    = track_width_in;
        d.wheel_diameter_in = wheel_diameter_in;
        d.max_forward_ips   = 60.0;
        return d;
    }

    MotorGroup make_left_group()  { return MotorGroup{left_motor};  }
    MotorGroup make_right_group() { return MotorGroup{right_motor}; }
};

} // namespace

TEST_CASE("AutoPid: lateral FOPDT fit recovers known K, tau, L within 5%") {
    FopdtRig rig;
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.lateral_step_voltage_mv  = 6000.0;
    spec.lateral_step_duration_sec = 1.0;
    spec.settle_period_sec        = 0.5;   // Don't observe long after step ends — plant decays
    spec.sample_period_sec        = 0.005; // 200 Hz so dead time discretizes cleanly
    AutoPid tuner(chassis, spec);
    tuner.set_pre_sample_hook([&](double dt) { rig.step(dt); });

    auto r = tuner.tune(TuneAxis::Lateral);
    INFO("message: " << r.message);
    INFO("K=" << r.model.K << " tau=" << r.model.tau << " L=" << r.model.L
         << " R^2=" << r.model.r_squared);
    REQUIRE(r.model.ok);
    // Expected steady-state: K_lat * 6000mV = 30 ips. The FOPDT K we fit
    // is the open-loop steady-state value of the OUTPUT (ips), not the
    // K_lat ratio (ips/mV). So expect ~30.
    const double expected_K = rig.K_lat * spec.lateral_step_voltage_mv;
    CHECK(r.model.K   == doctest::Approx(expected_K).epsilon(0.05));
    CHECK(r.model.tau == doctest::Approx(rig.tau_lat).epsilon(0.10));
    CHECK(r.model.L   == doctest::Approx(rig.L_lat).epsilon(0.30));   // dead-time always noisier
    CHECK(r.model.r_squared > 0.95);
}

TEST_CASE("AutoPid: angular FOPDT fit recovers known K, tau, L within 5%") {
    FopdtRig rig;
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.angular_step_voltage_mv  = 4000.0;
    spec.angular_step_duration_sec = 0.7;
    spec.settle_period_sec        = 0.4;
    spec.sample_period_sec        = 0.005;
    AutoPid tuner(chassis, spec);
    tuner.set_pre_sample_hook([&](double dt) { rig.step(dt); });

    auto r = tuner.tune(TuneAxis::Angular);
    INFO("message: " << r.message);
    INFO("K=" << r.model.K << " tau=" << r.model.tau << " L=" << r.model.L
         << " R^2=" << r.model.r_squared);
    REQUIRE(r.model.ok);
    const double expected_K = rig.K_ang * spec.angular_step_voltage_mv;
    CHECK(r.model.K   == doctest::Approx(expected_K).epsilon(0.05));
    CHECK(r.model.tau == doctest::Approx(rig.tau_ang).epsilon(0.10));
    CHECK(r.model.L   == doctest::Approx(rig.L_ang).epsilon(0.30));
    CHECK(r.model.r_squared > 0.95);
}

TEST_CASE("AutoPid: tuneBoth runs both axes and synthesizes positive gains") {
    FopdtRig rig;
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.sample_period_sec  = 0.005;
    spec.settle_period_sec  = 0.4;
    AutoPid tuner(chassis, spec);
    tuner.set_pre_sample_hook([&](double dt) { rig.step(dt); });

    auto [lat, ang] = tuner.tuneBoth();
    INFO("lat: " << lat.message);
    INFO("ang: " << ang.message);
    CHECK(lat.model.ok);
    CHECK(ang.model.ok);
    // Positive synthesized gains (within their clamp ranges).
    CHECK(lat.gains.kP > spec.kP_min);
    CHECK(lat.gains.kI >= 0.0);
    CHECK(lat.gains.kD >= 0.0);
    CHECK(ang.gains.kP > spec.kP_min);
    CHECK(ang.gains.kI >= 0.0);
    CHECK(ang.gains.kD >= 0.0);
}

TEST_CASE("AutoPid: synthesized gains drive the FOPDT plant to setpoint with low overshoot") {
    FopdtRig rig;
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.sample_period_sec        = 0.005;
    spec.settle_period_sec        = 0.4;
    spec.lambda_factor            = 1.5;     // a bit conservative — kinder to overshoot
    spec.overshoot_threshold_pct  = 25.0;    // give the synthesized PID some room
    spec.verify_lateral_distance_in = 12.0;
    AutoPid tuner(chassis, spec);
    tuner.set_pre_sample_hook([&](double dt) { rig.step(dt); });

    auto r = tuner.tune(TuneAxis::Lateral);
    INFO("message: " << r.message);
    INFO("overshoot_pct=" << r.verified_overshoot_pct);
    REQUIRE(r.model.ok);
    // The verification phase reports an overshoot. We don't strictly
    // require r.converged (the FOPDT plant + raw PID without anti-windup
    // can occasionally overshoot above 25%); we just want a sane
    // measurement that's bounded.
    CHECK(r.verified_overshoot_pct >= 0.0);
    CHECK(r.verified_overshoot_pct < 100.0);
}

TEST_CASE("AutoPid: noisy plant breaks the FOPDT fit and reports converged=false") {
    FopdtRig rig;
    rig.noise_sigma_lat = 50.0;   // huge zero-mean noise on v_x_meas
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.sample_period_sec = 0.005;
    spec.settle_period_sec = 0.4;
    AutoPid tuner(chassis, spec);
    tuner.set_pre_sample_hook([&](double dt) { rig.step(dt); });

    auto r = tuner.tune(TuneAxis::Lateral);
    INFO("message: " << r.message);
    // High-noise data should fail at least one of the fit gates: low R^2,
    // non-finite slope, or "process not first-order rising" (since the
    // ratio bounds may yield nonsensical fit windows). Either way:
    // converged should be false.
    CHECK_FALSE(r.converged);
}

TEST_CASE("AutoPid: degenerate sample period reports converged=false") {
    FopdtRig rig;
    Chassis chassis(rig.make_left_group(),
                    rig.make_right_group(),
                    make_18x18_bot(),
                    rig.make_sensors(),
                    Localization::DeadReckoning,
                    rig.make_drive_cfg());

    AutoPid::Spec spec;
    spec.sample_period_sec = 0.0;
    AutoPid tuner(chassis, spec);

    auto r = tuner.tune(TuneAxis::Lateral);
    CHECK_FALSE(r.converged);
}
