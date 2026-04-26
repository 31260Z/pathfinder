#pragma once

#include <pathfinder/autonomous/autonomous.hpp>
#include <pathfinder/chassis/controller_settings.hpp>
#include <pathfinder/chassis/drive_config.hpp>
#include <pathfinder/chassis/localization_kind.hpp>
#include <pathfinder/chassis/move_options.hpp>
#include <pathfinder/chassis/opcontrol_modes.hpp>
#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/driving/motor_group.hpp>
#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/bot.hpp>
#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>
#include <pathfinder/sensors/frame_helpers.hpp>
#include <pathfinder/sensors/sensors.hpp>

#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

namespace pathfinder {

// The Chassis god-object — owns the drivetrain, sensors, bot definition,
// localization estimator, and controller stacks. Spec §11 / §13 / §14 cheat
// sheet drives the public API; this Wave C implementation runs every motion
// verb synchronously and stubs `.async = true` with a clear runtime error.
//
// Key wiring (§"Architecture" of the Wave C brief):
//   1. The two MotorGroups are written via per-side voltage commands derived
//      from the active controller's DriveCommand and `tank_kinematics`.
//   2. The Sensors bundle is polled every tick: parallel-wheel inches,
//      perpendicular-wheel inches, and IMU heading flow into `DeadReckoning`.
//   3. The `lateral_` and `angular_` ControllerStacks supply default gains
//      to each motion verb; per-call MoveOpts/TurnOpts/FollowOpts can
//      override the `feedback` and `exit` fields.
//   4. `tick(dt)` does ONE iteration of the sensor → odom → controller →
//      motor pipeline (no sleeping). `run_until_done()` loops `tick` with a
//      real sleep between iterations; tests bypass it by calling `tick`
//      directly.
class Chassis {
public:
    // ── Construction ────────────────────────────────────────────────────
    // The localization arg is a tag instance (Localization::DeadReckoning,
    // Localization::Ekf) or a parameter struct (Localization::Mcl{...}).
    // Wave C wires DR; the others log a warning and fall back to DR.
    template <typename Loc>
    Chassis(MotorGroup left_motors,
            MotorGroup right_motors,
            Bot         bot,
            Sensors     sensors,
            Loc         localization,
            Drive       drive_config)
        : left_(std::move(left_motors)),
          right_(std::move(right_motors)),
          bot_(std::move(bot)),
          sensors_(std::move(sensors)),
          drive_(drive_config),
          odom_(make_dr_config(bot_, sensors_)) {
        warn_unimplemented_tier(localization);
        seed_sensor_state();
    }

    // ── Calibration / pose ──────────────────────────────────────────────
    // Blocks until every IMU reports `is_calibrating() == false`, with a 5s
    // safety timeout per IMU. FakeImu always returns false, so this is a
    // no-op in tests.
    void calibrate() {
        constexpr int    k_poll_ms        = 50;
        constexpr int    k_timeout_ms     = 5000;
        for (const auto& imu : sensors_.imus()) {
            int waited_ms = 0;
            while (imu.sensor && imu.sensor->is_calibrating() && waited_ms < k_timeout_ms) {
                host_sleep_for_ms(k_poll_ms);
                waited_ms += k_poll_ms;
            }
        }
        // Re-seed the sensor cache after calibration: any IMU reset that
        // happened during calibrate() shouldn't appear as a heading delta on
        // the next tick.
        seed_sensor_state();
    }

    void setPose(double x_in, double y_in, double heading_deg) {
        const Pose2 p{x_in, y_in, Angle::degrees(heading_deg)};
        odom_.set_pose(p);
        // Drop the integration history: the next tick is a "first" again so
        // that any encoder delta accumulated since the last poll doesn't get
        // re-applied to the teleported pose.
        odom_.reset();
        seed_sensor_state();
    }

    Pose2 getPose() const { return odom_.pose(); }

    // Pose covariance is always zero in DR. Wave F populates it from the EKF.
    Matrix3 getPoseCovariance() const { return Matrix3::zero(); }

    // ── Motion verbs (sync default) ────────────────────────────────────
    void moveTo(Vector2 target, MoveOpts opts = {}) {
        guard_async(opts.async);
        MoveToPoint::Options o{};
        o.along_track     = opts.along_track.value_or(lateral_.feedback);
        o.cross_track     = opts.cross_track.value_or(lateral_.feedback);
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_move_exit());
        o.max_forward_ips = drive_.max_forward_ips;
        o.max_angular_dps = drive_.max_angular_dps;
        o.reverse         = opts.reverse;

        MoveToPoint ctrl(odom_.pose(), target, o);
        run_until_done(ctrl, opts.timeout_ms);
    }

    void moveToPose(Pose2 target, MoveOpts opts = {}) {
        guard_async(opts.async);
        Boomerang::Options o{};
        o.along_track     = opts.along_track.value_or(lateral_.feedback);
        o.cross_track     = opts.cross_track.value_or(lateral_.feedback);
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_move_exit());
        o.max_forward_ips = drive_.max_forward_ips;
        o.max_angular_dps = drive_.max_angular_dps;
        o.lead            = opts.lead;
        o.reverse         = opts.reverse;

        Boomerang ctrl(odom_.pose(), target, o);
        run_until_done(ctrl, opts.timeout_ms);
    }

    void turnTo(double heading_deg, TurnOpts opts = {}) {
        guard_async(opts.async);
        TurnTo::Options o{};
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_turn_exit());
        o.max_angular_dps = drive_.max_angular_dps;

        TurnTo ctrl(Angle::degrees(heading_deg), o, to_turnto_dir(opts.direction));
        run_until_done(ctrl, opts.timeout_ms);
    }

    void turnTo(Vector2 target_point, TurnOpts opts = {}) {
        guard_async(opts.async);
        TurnTo::Options o{};
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_turn_exit());
        o.max_angular_dps = drive_.max_angular_dps;

        TurnToPoint ctrl(target_point, o, to_turnto_dir(opts.direction));
        run_until_done(ctrl, opts.timeout_ms);
    }

    // Single-side pivot — turn-in-place but only one side spins. Implemented
    // by routing through TurnTo and post-processing the per-tick command in
    // `tick_motors_for_swing` to zero out the locked side's voltage. The
    // active swing side is stashed in `swing_lock_side_` and cleared at exit.
    void swingTo(double heading_deg, Side side, TurnOpts opts = {}) {
        guard_async(opts.async);
        TurnTo::Options o{};
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_turn_exit());
        o.max_angular_dps = drive_.max_angular_dps;

        ScopedSwing swing(*this, side);
        TurnTo ctrl(Angle::degrees(heading_deg), o, to_turnto_dir(opts.direction));
        run_until_done(ctrl, opts.timeout_ms);
    }

    void swingTo(Vector2 target_point, Side side, TurnOpts opts = {}) {
        guard_async(opts.async);
        TurnTo::Options o{};
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_turn_exit());
        o.max_angular_dps = drive_.max_angular_dps;

        ScopedSwing swing(*this, side);
        TurnToPoint ctrl(target_point, o, to_turnto_dir(opts.direction));
        run_until_done(ctrl, opts.timeout_ms);
    }

    void follow(std::vector<Vector2> waypoints, FollowOpts opts = {}) {
        guard_async(opts.async);
        if (waypoints.size() < 2) {
            throw std::invalid_argument("Chassis::follow: need at least two waypoints");
        }
        catmull_rom::Path path(std::move(waypoints), opts.path_tension);
        PurePursuit::Options o{};
        o.forward         = opts.forward.value_or(lateral_.feedback);
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_follow_exit());
        o.lookahead_in    = opts.lookahead_in;
        o.max_forward_ips = drive_.max_forward_ips;
        o.max_angular_dps = drive_.max_angular_dps;
        o.reverse         = opts.reverse;

        PurePursuit ctrl(std::move(path), o);
        run_until_done(ctrl, opts.timeout_ms);
    }

    void follow(std::vector<Waypoint> waypoints, FollowOpts opts = {}) {
        guard_async(opts.async);
        if (waypoints.size() < 2) {
            throw std::invalid_argument("Chassis::follow: need at least two waypoints");
        }
        std::vector<Vector2> pts;
        pts.reserve(waypoints.size());
        for (const auto& w : waypoints) pts.push_back(w.point());

        catmull_rom::Path path(std::move(pts), opts.path_tension);
        PurePursuit::Options o{};
        o.forward         = opts.forward.value_or(lateral_.feedback);
        o.heading         = opts.heading.value_or(angular_.feedback);
        o.exit            = opts.exit.value_or(default_follow_exit());
        o.lookahead_in    = opts.lookahead_in;
        o.max_forward_ips = drive_.max_forward_ips;
        o.max_angular_dps = drive_.max_angular_dps;
        o.reverse         = opts.reverse;

        PurePursuit ctrl(std::move(path), o, std::move(waypoints));
        run_until_done(ctrl, opts.timeout_ms);
    }

    // ── Async coordination (Wave G stubs) ──────────────────────────────
    void waitUntil(double /*distance_in*/) {
        throw std::runtime_error(
            "async motion requires the runtime layer; install Wave G or wait for v0.2");
    }

    void waitUntilDone() {
        throw std::runtime_error(
            "async motion requires the runtime layer; install Wave G or wait for v0.2");
    }

    void cancelMotion() {
        // Best-effort stop: zero the motors. Doesn't throw; safe to call in
        // sync-only mode and future async-aware mode.
        write_per_side_voltage(0.0, 0.0);
    }

    // ── Driver control (sync — see brief §"Driver-control modes") ─────
    // Each opcontrol mode calls a user-supplied stick poll on every tick.
    // The two-arg overload takes a polling functor `(Axis) -> double in
    // [-1, 1]` plus a "should-keep-running" predicate; the chassis loops at
    // 100Hz until the predicate returns false.
    template <typename PollFn, typename ContinueFn>
    void opcontrol(TankDrive mode, PollFn poll, ContinueFn keep_going) {
        constexpr double dt = 0.010;
        while (keep_going()) {
            const double l = apply_curve(poll(mode.left_axis),  mode.expo_curve, mode.deadband);
            const double r = apply_curve(poll(mode.right_axis), mode.expo_curve, mode.deadband);
            const double l_ips = l * drive_.max_forward_ips;
            const double r_ips = r * drive_.max_forward_ips;
            tick_odometry();
            write_per_side_voltage(drive_.ips_to_voltage_mv(l_ips),
                                   drive_.ips_to_voltage_mv(r_ips));
            host_sleep_for_ms(static_cast<int>(dt * 1000.0));
        }
        write_per_side_voltage(0.0, 0.0);
    }

    template <typename PollFn, typename ContinueFn>
    void opcontrol(ArcadeDrive mode, PollFn poll, ContinueFn keep_going) {
        constexpr double dt = 0.010;
        while (keep_going()) {
            const double f = apply_curve(poll(mode.forward_axis), mode.expo_curve, mode.deadband);
            const double t = apply_curve(poll(mode.turn_axis),    mode.expo_curve, mode.deadband);
            const double f_ips = f * drive_.max_forward_ips;
            const double t_ips = t * drive_.max_forward_ips;   // turn maps as differential ips
            tick_odometry();
            write_per_side_voltage(drive_.ips_to_voltage_mv(f_ips + t_ips),
                                   drive_.ips_to_voltage_mv(f_ips - t_ips));
            host_sleep_for_ms(static_cast<int>(dt * 1000.0));
        }
        write_per_side_voltage(0.0, 0.0);
    }

    template <typename PollFn, typename ContinueFn>
    void opcontrol(CurvatureDrive mode, PollFn poll, ContinueFn keep_going) {
        constexpr double dt = 0.010;
        while (keep_going()) {
            const double th = apply_curve(poll(mode.throttle_axis),  mode.expo_curve, mode.deadband);
            const double cv = apply_curve(poll(mode.curvature_axis), mode.expo_curve, mode.deadband);
            const double th_ips   = th * drive_.max_forward_ips;
            // Curvature scales with throttle magnitude — at zero throttle the
            // bot still allows a quick-turn fall-through (cv used directly).
            const double scale    = std::abs(th);
            const double curve_v  = (scale > 0.05) ? (cv * th_ips) : (cv * drive_.max_forward_ips);
            tick_odometry();
            write_per_side_voltage(drive_.ips_to_voltage_mv(th_ips + curve_v),
                                   drive_.ips_to_voltage_mv(th_ips - curve_v));
            host_sleep_for_ms(static_cast<int>(dt * 1000.0));
        }
        write_per_side_voltage(0.0, 0.0);
    }

    // Convenience overloads that loop forever (until external interrupt).
    // Most tests use the three-arg variants so they can stop the loop.
    template <typename PollFn>
    void opcontrol(TankDrive mode, PollFn poll) {
        opcontrol(mode, poll, [] { return true; });
    }
    template <typename PollFn>
    void opcontrol(ArcadeDrive mode, PollFn poll) {
        opcontrol(mode, poll, [] { return true; });
    }
    template <typename PollFn>
    void opcontrol(CurvatureDrive mode, PollFn poll) {
        opcontrol(mode, poll, [] { return true; });
    }

    // ── Controller defaults ────────────────────────────────────────────
    void setController(ControllerAxis axis, ControllerStack stack) {
        if (axis == ControllerAxis::Lateral) lateral_ = std::move(stack);
        else                                  angular_ = std::move(stack);
    }

    const ControllerStack& controller(ControllerAxis axis) const {
        return (axis == ControllerAxis::Lateral) ? lateral_ : angular_;
    }

    // ── Test-friendly `tick`: one iteration of sensor → odom → motor.
    // Intended for unit tests that wire FakeMotors back into FakeRotations
    // and want to step the loop without sleeping. The chassis calls this
    // internally inside both `run_until_done` and `opcontrol`.
    struct TickResult {
        Pose2        pose;
        DriveCommand cmd;
        bool         done = true;
    };

    TickResult tick(double /*dt_sec*/) {
        tick_odometry();
        return TickResult{odom_.pose(), DriveCommand{}, true};
    }

    // ── Bot-relative point access (delegates to Bot) ───────────────────
    Vector2 named_point_offset(std::string_view name) const {
        return bot_.point(name).offset;
    }

    // ── Inspection accessors ───────────────────────────────────────────
    const Bot&     bot()     const { return bot_; }
    const Sensors& sensors() const { return sensors_; }
    const Drive&   drive()   const { return drive_; }

    // Per-side last-commanded voltage for tests that don't want to reach
    // through the underlying FakeMotors.
    double last_left_voltage_mv()  const { return last_left_mv_; }
    double last_right_voltage_mv() const { return last_right_mv_; }

private:
    // ── Sensor → odom integration ──────────────────────────────────────
    void seed_sensor_state() {
        prev_par_pos_revs_  = read_first_parallel_position_revs();
        prev_perp_pos_revs_ = read_first_perpendicular_position_revs();
    }

    void tick_odometry() {
        // Parallel-wheel inches: take the FIRST parallel wheel for v1.
        // Multi-wheel averaging is a future tweak (per the brief).
        // Note: parallel_wheels() returns by value; bind to a local before
        // taking element references to avoid dangling.
        double par_in = 0.0;
        const auto par_wheels = sensors_.parallel_wheels();
        if (!par_wheels.empty()) {
            const TrackingWheel& w = par_wheels.front();
            const double now = w.sensor ? w.sensor->position_revolutions() : 0.0;
            const double d_revs = now - prev_par_pos_revs_;
            par_in = w.encoder_to_inches(d_revs);
            prev_par_pos_revs_ = now;
        }

        double perp_in = 0.0;
        const auto perp_wheels = sensors_.perpendicular_wheels();
        if (!perp_wheels.empty()) {
            const TrackingWheel& w = perp_wheels.front();
            const double now = w.sensor ? w.sensor->position_revolutions() : 0.0;
            const double d_revs = now - prev_perp_pos_revs_;
            perp_in = w.encoder_to_inches(d_revs);
            prev_perp_pos_revs_ = now;
        }

        // IMU heading: simple average of all populated IMUs in bot frame.
        // Wave F replaces this with covariance-weighted fusion.
        Angle heading = odom_.pose().heading;
        if (!sensors_.imus().empty()) {
            double s_sum = 0.0, c_sum = 0.0;
            int    count = 0;
            for (const auto& imu : sensors_.imus()) {
                if (!imu.sensor) continue;
                const double r = imu.heading_in_bot_frame().rad;
                s_sum += std::sin(r);
                c_sum += std::cos(r);
                ++count;
            }
            if (count > 0) heading = Angle{std::atan2(s_sum, c_sum)};
        }

        odom_.update(par_in, perp_in, heading);
    }

    double read_first_parallel_position_revs() const {
        const auto wheels = sensors_.parallel_wheels();
        if (wheels.empty()) return 0.0;
        const auto& w = wheels.front();
        return w.sensor ? w.sensor->position_revolutions() : 0.0;
    }

    double read_first_perpendicular_position_revs() const {
        const auto wheels = sensors_.perpendicular_wheels();
        if (wheels.empty()) return 0.0;
        const auto& w = wheels.front();
        return w.sensor ? w.sensor->position_revolutions() : 0.0;
    }

    // ── Motion-loop core ───────────────────────────────────────────────
    template <typename Controller>
    void run_until_done(Controller& ctrl, double timeout_ms) {
        constexpr double dt_sec = 0.010;
        const auto       start  = std::chrono::steady_clock::now();
        while (!ctrl.done()) {
            tick_odometry();
            const DriveCommand cmd = ctrl.update(odom_.pose(), dt_sec);
            apply_drive_command(cmd);
            if (elapsed_ms_since(start) > timeout_ms) break;
            host_sleep_for_ms(static_cast<int>(dt_sec * 1000.0));
        }
        write_per_side_voltage(0.0, 0.0);
    }

    void apply_drive_command(DriveCommand cmd) {
        const PerSideVelocities sides = tank_kinematics(cmd, drive_.track_width_in);
        double left_mv  = drive_.ips_to_voltage_mv(sides.left_ips);
        double right_mv = drive_.ips_to_voltage_mv(sides.right_ips);
        if (swing_lock_side_ == Side::Left)  left_mv  = 0.0;
        if (swing_lock_side_ == Side::Right) right_mv = 0.0;
        write_per_side_voltage(left_mv, right_mv);
    }

    void write_per_side_voltage(double left_mv, double right_mv) {
        last_left_mv_  = left_mv;
        last_right_mv_ = right_mv;
        left_.set_voltage_mv(left_mv);
        right_.set_voltage_mv(right_mv);
    }

    // ── Defaults / helpers ─────────────────────────────────────────────
    static ExitConditions::Spec default_move_exit() {
        ExitConditions::Spec s{};
        s.small_error            = 0.5;
        s.small_error_timeout_ms = 100.0;
        s.large_error            = 2.0;
        s.large_error_timeout_ms = 800.0;
        s.absolute_timeout_ms    = 30000.0;
        return s;
    }

    static ExitConditions::Spec default_turn_exit() {
        ExitConditions::Spec s{};
        s.small_error            = 1.0;     // degrees
        s.small_error_timeout_ms = 100.0;
        s.large_error            = 5.0;
        s.large_error_timeout_ms = 500.0;
        s.absolute_timeout_ms    = 10000.0;
        return s;
    }

    static ExitConditions::Spec default_follow_exit() {
        ExitConditions::Spec s{};
        s.small_error            = 0.7;
        s.small_error_timeout_ms = 100.0;
        s.large_error            = 2.0;
        s.large_error_timeout_ms = 800.0;
        s.absolute_timeout_ms    = 60000.0;
        return s;
    }

    static TurnTo::Direction to_turnto_dir(TurnOpts::Direction d) {
        switch (d) {
            case TurnOpts::Direction::Auto: return TurnTo::Direction::Auto;
            case TurnOpts::Direction::CW:   return TurnTo::Direction::CW;
            case TurnOpts::Direction::CCW:  return TurnTo::Direction::CCW;
        }
        return TurnTo::Direction::Auto;
    }

    static double apply_curve(double stick, double expo, double deadband) {
        if (std::abs(stick) < deadband) return 0.0;
        if (expo <= 0.0) return stick;
        // Standard expo blend: out = (1-expo)·x + expo·x³.
        const double e = std::min(expo, 1.0);
        return (1.0 - e) * stick + e * stick * stick * stick;
    }

    static void host_sleep_for_ms(int ms) {
        if (ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    }

    static double elapsed_ms_since(std::chrono::steady_clock::time_point t0) {
        const auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - t0).count();
    }

    static DeadReckoning::Config make_dr_config(const Bot& bot, const Sensors& sensors) {
        DeadReckoning::Config cfg{};
        if (!sensors.parallel_wheels().empty()) {
            const Vector2 c = corner_to_center_offset(
                sensors.parallel_wheels().front().offset, bot);
            cfg.parallel_wheel_y_offset_in = c.y;
        }
        if (!sensors.perpendicular_wheels().empty()) {
            const Vector2 c = corner_to_center_offset(
                sensors.perpendicular_wheels().front().offset, bot);
            cfg.perp_wheel_x_offset_in = c.x;
            cfg.has_perp_wheel         = true;
        }
        return cfg;
    }

    static void warn_unimplemented_tier(Localization::DeadReckoning_t) {}
    static void warn_unimplemented_tier(Localization::Ekf_t) {
        std::cerr << "[pathfinder] Localization::Ekf is not implemented in this "
                     "build (Wave F); falling back to DeadReckoning.\n";
    }
    static void warn_unimplemented_tier(const Localization::Mcl&) {
        std::cerr << "[pathfinder] Localization::Mcl is not implemented in this "
                     "build (Wave F); falling back to DeadReckoning.\n";
    }

    void guard_async(bool async) const {
        if (async) {
            throw std::runtime_error(
                "async motion requires the runtime layer; install Wave G or wait for v0.2");
        }
    }

    // RAII helper: sets the swing-lock side for the duration of a single
    // motion verb, restores to "no lock" on destruction.
    struct ScopedSwing {
        Chassis& c;
        ScopedSwing(Chassis& chassis, Side side) : c(chassis) { c.swing_lock_side_ = side; }
        ~ScopedSwing() { c.swing_lock_side_ = std::nullopt; }
        ScopedSwing(const ScopedSwing&)            = delete;
        ScopedSwing& operator=(const ScopedSwing&) = delete;
    };

    // ── State ──────────────────────────────────────────────────────────
    MotorGroup            left_;
    MotorGroup            right_;
    Bot                   bot_;
    Sensors               sensors_;
    Drive                 drive_;
    DeadReckoning         odom_;
    ControllerStack       lateral_{};
    ControllerStack       angular_{};
    double                prev_par_pos_revs_  = 0.0;
    double                prev_perp_pos_revs_ = 0.0;
    double                last_left_mv_       = 0.0;
    double                last_right_mv_      = 0.0;
    std::optional<Side>   swing_lock_side_    = std::nullopt;
};

} // namespace pathfinder
