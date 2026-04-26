#include <doctest/doctest.h>

#include <pathfinder/calibration/wheel_finder.hpp>
#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/sensors/mocks.hpp>
#include <pathfinder/sensors/tracking_wheel.hpp>

#include "../chassis/sim_helpers.hpp"

#include <cmath>
#include <memory>
#include <vector>

using namespace pathfinder;
using pathfinder::calibration::WheelFinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;

namespace {

// ── Synthetic pivot rig ────────────────────────────────────────────────
//
// We model a fixed pivot point P (in bot-frame) and one or more tracking
// wheels at known offsets. On each pre_sample_hook tick we advance the
// IMU heading by Δθ_per_tick and update each wheel's encoder by the
// distance dictated by the App. A formula:
//   parallel  Δs = (P_y − y_p) · Δθ_total
//   perp      Δs = (x_q − P_x) · Δθ_total
// We integrate per-tick deltas to keep the wheel revs and IMU heading in
// lockstep with the calibration loop's sampling.
struct PivotRig {
    // Sensor objects exposed to the test for sensor-bundle wiring.
    std::shared_ptr<FakeImu>      imu = std::make_shared<FakeImu>();

    struct Wheel {
        std::shared_ptr<FakeRotation> rot = std::make_shared<FakeRotation>();
        Vector2 offset_in;           // (x_p, y_p) in bot frame
        Axis    axis;                // X = parallel, Y = perpendicular
        double  diameter_in = 2.75;  // for inches → revs conversion
    };
    std::vector<Wheel> wheels;

    Vector2 pivot{0.0, 0.0};
    double  delta_theta_per_tick_rad = 0.0;
    double  total_theta_rad          = 0.0;
    int     ticks_remaining          = 0;   // after this, we hold heading still

    // Reset internal accumulators (call between pivots).
    // `keep_heading=true` (default) preserves the FakeImu heading across
    // pivots; the synthetic rotation just continues from where it was.
    // This matters when the rig is driven by `WheelFinder::run_all_corners`
    // — that loop never resets the chassis heading, so a discontinuous
    // jump in the IMU reading would inject a spurious Δθ into the very
    // first tick of the new pivot.
    void reset_for_pivot(Vector2 new_pivot, double dtheta_per_tick, int n_active_ticks,
                         bool keep_heading = true) {
        pivot                    = new_pivot;
        delta_theta_per_tick_rad = dtheta_per_tick;
        ticks_remaining          = n_active_ticks;
        if (!keep_heading) {
            total_theta_rad = 0.0;
            imu->set_heading(Angle{0.0});
        }
        // Don't reset wheel positions — keep accumulating across pivots so
        // we exercise the start-vs-end snapshot logic in WheelFinder.
    }

    void step(double /*dt*/) {
        if (ticks_remaining <= 0) return;
        const double prev_theta = total_theta_rad;
        total_theta_rad += delta_theta_per_tick_rad;
        const double dtheta_inc = total_theta_rad - prev_theta;
        for (auto& w : wheels) {
            const double d_in = (w.axis == Axis::X)
                ? (pivot.y - w.offset_in.y) * dtheta_inc
                : (w.offset_in.x - pivot.x) * dtheta_inc;
            const double circ = k_pi * w.diameter_in;
            if (circ > 0.0) {
                w.rot->set_position(w.rot->position_revolutions() + d_in / circ);
            }
        }
        imu->set_heading(Angle{total_theta_rad});
        --ticks_remaining;
    }
};

// Build a Sensors bundle backed by a PivotRig.
Sensors make_sensors_from_pivot(PivotRig& rig) {
    Sensors s;
    for (auto& w : rig.wheels) {
        TrackingWheel tw{};
        tw.sensor             = w.rot;
        tw.wheel              = Wheel::Custom;
        tw.custom_diameter_in = w.diameter_in;
        tw.axis               = w.axis;
        tw.offset             = w.offset_in;
        s.add(tw);
    }
    Imu imu_cfg{};
    imu_cfg.sensor       = rig.imu;
    imu_cfg.mounting     = ImuMounting::ZDown_XForward;   // sign +1
    imu_cfg.offset_xy_in = {9.0, 9.0};
    s.add(imu_cfg);
    return s;
}

// Drive defaults that match the SimRig's default geometry but don't
// depend on any motor input (WheelFinder doesn't command motors).
Drive default_drive() {
    Drive d{};
    d.track_width_in    = 12.0;
    d.wheel_diameter_in = 3.25;
    d.max_forward_ips   = 60.0;
    return d;
}

} // namespace

TEST_CASE("WheelFinder: recovers a known parallel-wheel y-offset from a single pivot") {
    PivotRig rig;
    // One parallel wheel at (4, -3) in bot frame.
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {4.0, -3.0}, Axis::X, 2.75});

    SimRig sim_for_motors;   // unused for motor IO; the chassis still wants MotorGroups
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    // BackLeft pivot of an 18×18 bot with BackLeft origin: (0, 0).
    // Sweep ~3.5 rad over 350 ticks (10 mrad/tick at 100 Hz). Single
    // pivot — fresh heading is fine.
    rig.reset_for_pivot({0.0, 0.0}, 0.010, 350, /*keep_heading=*/false);

    WheelFinder finder(chassis);
    finder.set_pre_sample_hook([&](double t_sec) { rig.step(t_sec); });

    auto pr = finder.run_pivot(WheelFinder::PivotCorner::BackLeft);
    INFO("message: " << pr.message);
    INFO("delta_theta_rad: " << pr.delta_theta_rad);
    REQUIRE(pr.converged);
    REQUIRE(pr.parallel_y_estimates_in.size() == 1);
    // Recovered y_p should match the ground-truth -3.0 within tight bounds.
    CHECK(pr.parallel_y_estimates_in[0] == doctest::Approx(-3.0).epsilon(0.01));
}

TEST_CASE("WheelFinder: recovers a known perpendicular-wheel x-offset from a single pivot") {
    PivotRig rig;
    // One perpendicular wheel at (5, 9) in bot frame.
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {5.0, 9.0}, Axis::Y, 2.75});

    SimRig sim_for_motors;
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    rig.reset_for_pivot({0.0, 0.0}, 0.010, 350, /*keep_heading=*/false);

    WheelFinder finder(chassis);
    finder.set_pre_sample_hook([&](double t_sec) { rig.step(t_sec); });

    auto pr = finder.run_pivot(WheelFinder::PivotCorner::BackLeft);
    INFO("message: " << pr.message);
    REQUIRE(pr.converged);
    REQUIRE(pr.perpendicular_x_estimates_in.size() == 1);
    // Recovered x_q matches the ground-truth 5.0.
    CHECK(pr.perpendicular_x_estimates_in[0] == doctest::Approx(5.0).epsilon(0.01));
}

TEST_CASE("WheelFinder: aggregate over four corners reduces variance to <0.05 in") {
    PivotRig rig;
    // Two wheels: one parallel at (3, 2), one perpendicular at (1, 6).
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {3.0, 2.0}, Axis::X, 2.75});
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {1.0, 6.0}, Axis::Y, 2.75});

    SimRig sim_for_motors;
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    // Iteration order in WheelFinder::run_all_corners is BackLeft,
    // FrontLeft, FrontRight, BackRight. Corner coords on an 18×18 BackLeft-
    // origin bot: BL (0,0), FL (18,0), FR (18,18), BR (0,18). We need the
    // hook to switch the pivot at the start of each pivot call.
    int call_index = 0;
    const Vector2 pivots_in_order[] = {
        {0.0, 0.0},   // BackLeft
        {18.0, 0.0},  // FrontLeft
        {18.0, 18.0}, // FrontRight
        {0.0, 18.0},  // BackRight
    };

    WheelFinder finder(chassis);
    finder.set_pre_sample_hook([&](double t_sec) {
        // Detect a "fresh pivot" by the IMU heading being zero AND ticks
        // already exhausted from the previous pivot. WheelFinder
        // applyVoltage(0,0)s and snaps the IMU and wheel positions BEFORE
        // calling the hook on the first tick of a new pivot — but we can
        // just unconditionally rearm at every transition triggered from
        // outside the loop.
        rig.step(t_sec);
    });

    // We need to arm each pivot before WheelFinder calls run_pivot on it.
    // Easiest: do it manually with run_pivot in the right order.
    WheelFinder::AggregateResult agg;
    const WheelFinder::PivotCorner k_order[] = {
        WheelFinder::PivotCorner::BackLeft,
        WheelFinder::PivotCorner::FrontLeft,
        WheelFinder::PivotCorner::FrontRight,
        WheelFinder::PivotCorner::BackRight,
    };
    std::vector<WheelFinder::PivotResult> per_pivot;
    for (int i = 0; i < 4; ++i) {
        // First pivot starts fresh; later pivots keep heading continuous
        // so WheelFinder's `start_heading_rad` snapshot agrees with the
        // first sampled IMU reading.
        rig.reset_for_pivot(pivots_in_order[i], 0.010, 350,
                            /*keep_heading=*/(i > 0));
        per_pivot.push_back(finder.run_pivot(k_order[i]));
        ++call_index;
    }
    (void)call_index;

    // Aggregate by hand using the same arithmetic WheelFinder does (mean +
    // sample stddev across converged pivots).
    auto summarize = [](const std::vector<double>& xs) {
        double sum = 0.0;
        for (double x : xs) sum += x;
        const double mean = sum / static_cast<double>(xs.size());
        double sq = 0.0;
        for (double x : xs) sq += (x - mean) * (x - mean);
        const double std_dev = (xs.size() > 1)
            ? std::sqrt(sq / static_cast<double>(xs.size() - 1))
            : 0.0;
        return std::pair<double, double>{mean, std_dev};
    };

    std::vector<double> par_estimates, perp_estimates;
    for (const auto& pr : per_pivot) {
        REQUIRE(pr.converged);
        par_estimates.push_back(pr.parallel_y_estimates_in.at(0));
        perp_estimates.push_back(pr.perpendicular_x_estimates_in.at(0));
    }
    auto par_summary  = summarize(par_estimates);
    auto perp_summary = summarize(perp_estimates);
    CHECK(par_summary.first  == doctest::Approx(2.0).epsilon(0.02));
    CHECK(perp_summary.first == doctest::Approx(1.0).epsilon(0.02));
    CHECK(par_summary.second  < 0.05);
    CHECK(perp_summary.second < 0.05);
}

TEST_CASE("WheelFinder::run_all_corners aggregates per wheel across pivots") {
    // Routed through WheelFinder::run_all_corners. The hook gets the per-
    // pivot tick time (t_sec = 0 on the first tick of each pivot), so we
    // detect "new pivot started" by t_sec going back to 0 and step our
    // own pivot index at that boundary.
    PivotRig rig;
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {3.0, 2.0}, Axis::X, 2.75});
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {1.0, 6.0}, Axis::Y, 2.75});

    SimRig sim_for_motors;
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    // run_all_corners iterates BackLeft → FrontLeft → FrontRight → BackRight.
    // 18×18 BackLeft-origin coords:
    const Vector2 pivots_in_order[] = {
        {0.0, 0.0},   // BackLeft
        {18.0, 0.0},  // FrontLeft
        {18.0, 18.0}, // FrontRight
        {0.0, 18.0},  // BackRight
    };
    int pivot_idx = -1;
    bool prev_was_nonzero_t = false;

    WheelFinder::Spec spec;
    spec.timeout_per_pivot_sec = 5.0;   // 500 ticks at 10 ms — over our 350 active sweep
    WheelFinder finder(chassis, spec);
    finder.set_pre_sample_hook([&](double t_sec) {
        // First tick of a new pivot: t_sec == 0 and the previous pivot's
        // last tick had t_sec > 0. Re-arm the rig with the next corner.
        // Don't reset the chassis pose or IMU heading — WheelFinder snaps
        // start_heading_rad before the first hook fires; resetting the
        // IMU here would inject a 2π-class spurious Δθ on the first tick.
        if (t_sec == 0.0 && (prev_was_nonzero_t || pivot_idx < 0)) {
            ++pivot_idx;
            const bool is_first = (pivot_idx == 0);
            if (pivot_idx < 4) {
                rig.reset_for_pivot(pivots_in_order[pivot_idx], 0.010, 350,
                                    /*keep_heading=*/!is_first);
            }
        }
        prev_was_nonzero_t = (t_sec > 0.0);
        rig.step(t_sec);
    });

    auto agg = finder.run_all_corners();
    INFO("agg.message: " << agg.message);
    REQUIRE(agg.converged);
    REQUIRE(agg.parallel_wheels.size()      == 1);
    REQUIRE(agg.perpendicular_wheels.size() == 1);
    CHECK(agg.parallel_wheels[0].mean_offset_in == doctest::Approx(2.0).epsilon(0.02));
    CHECK(agg.perpendicular_wheels[0].mean_offset_in == doctest::Approx(1.0).epsilon(0.02));
    CHECK(agg.parallel_wheels[0].stddev_offset_in       < 0.05);
    CHECK(agg.perpendicular_wheels[0].stddev_offset_in  < 0.05);
    CHECK(agg.parallel_wheels[0].n_samples      == 4u);
    CHECK(agg.perpendicular_wheels[0].n_samples == 4u);
}

TEST_CASE("WheelFinder: insufficient rotation reports converged=false") {
    PivotRig rig;
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {4.0, -3.0}, Axis::X, 2.75});

    SimRig sim_for_motors;
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    // 30° = 0.524 rad — below the 3.0 rad min. Spread across 100 ticks so
    // the still-dwell detector won't trip mid-rotation.
    rig.reset_for_pivot({0.0, 0.0}, 0.524 / 100.0, 100);

    WheelFinder::Spec spec;
    spec.timeout_per_pivot_sec = 2.0;   // shorter timeout so the test runs fast
    WheelFinder finder(chassis, spec);
    finder.set_pre_sample_hook([&](double t_sec) { rig.step(t_sec); });

    auto pr = finder.run_pivot(WheelFinder::PivotCorner::BackLeft);
    CHECK_FALSE(pr.converged);
    INFO("message: " << pr.message);
    CHECK(pr.message.find("insufficient rotation") != std::string::npos);
}

TEST_CASE("WheelFinder: degenerate sample period reports converged=false") {
    PivotRig rig;
    rig.wheels.push_back({std::make_shared<FakeRotation>(), {0.0, 0.0}, Axis::X, 2.75});

    SimRig sim_for_motors;
    Chassis chassis(make_left_group(sim_for_motors),
                    make_right_group(sim_for_motors),
                    make_18x18_bot(),
                    make_sensors_from_pivot(rig),
                    Localization::DeadReckoning,
                    default_drive());

    WheelFinder::Spec spec;
    spec.sample_period_sec = 0.0;
    WheelFinder finder(chassis, spec);

    auto pr = finder.run_pivot(WheelFinder::PivotCorner::BackLeft);
    CHECK_FALSE(pr.converged);
}
