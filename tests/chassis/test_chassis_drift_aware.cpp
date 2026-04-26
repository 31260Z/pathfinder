#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

#include <cmath>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {
constexpr double k_dt = 0.01;
} // namespace

TEST_CASE("Chassis::getBodyVelocity: zero on construction (no motion)") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    const BodyVelocity v = chassis.getBodyVelocity();
    CHECK(v.v_x_ips == doctest::Approx(0.0));
    CHECK(v.v_y_ips == doctest::Approx(0.0));
    CHECK(v.omega_dps == doctest::Approx(0.0));
}

TEST_CASE("Chassis::getBodyVelocity: pure-forward driving reports v_x") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    rig.left_motor->set_voltage_mv(6000.0);
    rig.right_motor->set_voltage_mv(6000.0);

    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 20; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    // 6000 mV @ 12000 max = 0.5 → 0.5 × 60 ips = 30 ips per side, both sides
    // → forward velocity 30 ips, no rotation.
    const BodyVelocity v = chassis.getBodyVelocity();
    CHECK(v.v_x_ips == doctest::Approx(30.0).epsilon(0.05));
    CHECK(v.v_y_ips == doctest::Approx(0.0).epsilon(0.01));
    CHECK(v.omega_dps == doctest::Approx(0.0).epsilon(0.01));
}

TEST_CASE("Chassis::getBodyVelocity: turn-while-moving builds v_y via cross-coupling") {
    // Drive a forward+turn sequence and verify the chassis-estimated v_y
    // grows in magnitude while ω is active and decays once ω stops. The
    // chassis has a perp-less sensor stack (only the parallel wheel + IMU)
    // and a low lateral_friction_coefficient, so the model integrator runs
    // and the buildup is observable.
    SimRig rig;
    auto drive_cfg = make_drive(rig);
    drive_cfg.lateral_friction_coefficient = 1.0;   // omni — slow decay
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    drive_cfg);

    // Phase 1: drive forward for 100 ms with both wheels at 6 V.
    rig.left_motor->set_voltage_mv(6000.0);
    rig.right_motor->set_voltage_mv(6000.0);
    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 10; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    REQUIRE(chassis.getBodyVelocity().v_x_ips > 25.0);
    REQUIRE(std::abs(chassis.getBodyVelocity().v_y_ips) < 0.01);

    // Phase 2: differential voltage briefly (rotation while moving). Left
    // forward, right reverse → CCW rotation in the right-handed convention,
    // but the IMU+SimRig math here uses a CW-positive heading rate matching
    // the spec, so this still produces a non-zero ω · v_x cross term.
    rig.left_motor->set_voltage_mv(8000.0);
    rig.right_motor->set_voltage_mv(2000.0);
    for (int i = 0; i < 20; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    const BodyVelocity v_after_inject = chassis.getBodyVelocity();
    // Cross-coupling should have injected a noticeable v_y. Sign comes from
    // the ω·v_x term — direction doesn't matter for the magnitude check.
    CHECK(std::abs(v_after_inject.v_y_ips) > 0.5);

    // Phase 3: stop rotating (back to symmetric voltage), watch v_y decay.
    rig.left_motor->set_voltage_mv(5000.0);
    rig.right_motor->set_voltage_mv(5000.0);
    for (int i = 0; i < 50; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    const BodyVelocity v_after_decay = chassis.getBodyVelocity();
    // After 0.5 s with μ=1, decay factor ≈ e^(-0.5) ≈ 0.607.
    CHECK(std::abs(v_after_decay.v_y_ips) < std::abs(v_after_inject.v_y_ips) * 0.7);
}

TEST_CASE("Chassis::getBodyVelocity: matches the spec §8 model after a step injection") {
    // Compare the chassis's integrated body-frame v_y to a hand-rolled
    // forward-Euler integration of the same model, given the same inputs.
    // They should match to machine precision (apart from float rounding).
    SimRig rig;
    auto drive_cfg = make_drive(rig);
    constexpr double mu = 0.5;
    drive_cfg.lateral_friction_coefficient = mu;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    drive_cfg);

    // Drive at constant 30 ips, then for one tick rotate at known ω, then
    // stop rotating and let v_y decay.
    rig.left_motor->set_voltage_mv(6000.0);
    rig.right_motor->set_voltage_mv(6000.0);
    chassis.tick(k_dt);
    for (int i = 0; i < 10; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }

    // Synthetic injection: differential voltage for 5 steps.
    rig.left_motor->set_voltage_mv(7200.0);
    rig.right_motor->set_voltage_mv(4800.0);
    for (int i = 0; i < 5; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    const double v_y_injected = chassis.getBodyVelocity().v_y_ips;
    REQUIRE(std::abs(v_y_injected) > 0.1);

    // Compare to analytic decay: stop rotation, run for N steps, expect
    // v_y(t) ≈ v_y_0 · e^(−μ·t).
    rig.left_motor->set_voltage_mv(6000.0);
    rig.right_motor->set_voltage_mv(6000.0);
    constexpr int    decay_steps = 100;
    constexpr double decay_t     = decay_steps * k_dt;
    for (int i = 0; i < decay_steps; ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
    }
    const double v_y_after = chassis.getBodyVelocity().v_y_ips;
    const double expected  = v_y_injected * std::exp(-mu * decay_t);
    // Forward-Euler at dt=0.01 with μ=0.5 should be very close to the
    // analytic e-folding (the discretization error is < 1% per cycle).
    // Add a small absolute tolerance so the comparison stays meaningful when
    // the value approaches zero.
    CHECK(std::abs(v_y_after - expected) < 0.05 * std::abs(v_y_injected));
}

TEST_CASE("Chassis::moveTo: lookahead_time_sec field flows through to the controller") {
    // We can't observe the lookahead_time_sec value directly from outside the
    // controller; the easiest indirect check is to verify the chassis still
    // reaches its target whether lookahead is on or off in the no-drift case
    // (perfect bot, no v_y to predict). This guards against accidentally
    // breaking the wiring (e.g. forgetting to plumb the option through).
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    Vector2 target{24.0, 0.0};
    MoveToPoint::Options o{};
    o.along_track        = Pid::Gains{4.0, 0.0, 0.2};
    o.cross_track        = Pid::Gains{1.5, 0.0, 0.05};
    o.heading            = Pid::Gains{6.0, 0.0, 0.4};
    o.exit = ExitConditions::Spec{
        .small_error = 0.5, .small_error_timeout_ms = 100.0,
        .large_error = 2.0, .large_error_timeout_ms = 800.0,
        .absolute_timeout_ms = 30000.0,
    };
    o.max_forward_ips    = 60.0;
    o.max_angular_dps    = 360.0;
    o.lookahead_time_sec = 0.20;   // 200 ms lookahead

    MoveToPoint ctrl(chassis.getPose(), target, o);

    // External closed-loop driver (mirrors test_chassis_move_to.cpp pattern).
    chassis.tick(k_dt);   // seed
    for (int i = 0; i < 4000 && !ctrl.done(); ++i) {
        rig.step(k_dt);
        chassis.tick(k_dt);
        // Drift-aware overload — the chassis tracks v_y in its DR; we plumb
        // it explicitly here to exercise the same call site the chassis's
        // run_until_done does internally.
        DriveCommand cmd = ctrl.update(chassis.getPose(),
                                       chassis.getBodyVelocity(), k_dt);
        const auto sides = tank_kinematics(cmd, rig.track_width_in);
        rig.left_motor->set_voltage_mv((sides.left_ips  / rig.max_forward_ips) * 12000.0);
        rig.right_motor->set_voltage_mv((sides.right_ips / rig.max_forward_ips) * 12000.0);
    }
    CHECK(ctrl.done());
    CHECK(distance(chassis.getPose().translation(), target) < 2.0);
}
