#pragma once

#include <pathfinder/controllers/exit_conditions.hpp>
#include <pathfinder/controllers/pid.hpp>

#include <optional>

namespace pathfinder {

// Side enum re-used by `swingTo`. Must precede `TurnOpts` because the swing
// variants take it as a positional parameter.
enum class Side { Left, Right };

// Per-call options for `Chassis::moveTo` and `Chassis::moveToPose`.
// All gain / exit overrides are optional — when std::nullopt the chassis's
// current `ControllerStack` defaults are used.
struct MoveOpts {
    double timeout_ms         = 5000.0;
    bool   reverse            = false;
    bool   async              = false;
    double min_exit_speed_ips = 0.0;   // motion chaining hand-off speed (Wave D)
    double early_exit_in      = 0.0;   // chaining (Wave D placeholder)
    double lead               = 0.3;   // for moveToPose (Boomerang)
    // Drift-aware lookahead horizon (s). The cross-track regulator predicts
    // line offset `lookahead_time_sec` ahead given current v_y. 0 disables
    // drift-aware prediction. See spec §9. Default 0.10s — predict 100 ms
    // ahead (one tick at the controller's 10 ms cadence × 10).
    double lookahead_time_sec = 0.10;

    // PID gain overrides — std::nullopt = use chassis defaults
    std::optional<Pid::Gains> along_track;
    std::optional<Pid::Gains> cross_track;
    std::optional<Pid::Gains> heading;

    // ExitConditions override
    std::optional<ExitConditions::Spec> exit;
};

// Per-call options for the family of `turnTo` / `swingTo` verbs.
struct TurnOpts {
    double timeout_ms = 2000.0;
    bool   async      = false;

    enum class Direction { Auto, CW, CCW };
    Direction direction = Direction::Auto;

    std::optional<Pid::Gains>           heading;
    std::optional<ExitConditions::Spec> exit;
};

// Per-call options for `Chassis::follow`.
struct FollowOpts {
    double timeout_ms         = 5000.0;
    double lookahead_in       = 12.0;
    double path_tension       = 0.5;
    bool   reverse            = false;
    bool   async              = false;
    double min_exit_speed_ips = 0.0;   // motion chaining hand-off speed (Wave D)
    // Drift-aware lookahead horizon (s) — see MoveOpts.
    double lookahead_time_sec = 0.10;

    std::optional<Pid::Gains>           forward;
    std::optional<Pid::Gains>           heading;
    std::optional<ExitConditions::Spec> exit;
};

} // namespace pathfinder
