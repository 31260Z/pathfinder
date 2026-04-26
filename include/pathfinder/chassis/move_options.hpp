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
    double timeout_ms        = 5000.0;
    bool   reverse           = false;
    bool   async             = false;
    double min_exit_speed_ips = 0.0;   // for chaining (Wave D consumes this)
    double early_exit_in     = 0.0;    // for chaining (Wave D consumes this)
    double lead              = 0.3;    // for moveToPose (Boomerang)

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
    double timeout_ms   = 5000.0;
    double lookahead_in = 12.0;
    double path_tension = 0.5;
    bool   reverse      = false;
    bool   async        = false;

    std::optional<Pid::Gains>           forward;
    std::optional<Pid::Gains>           heading;
    std::optional<ExitConditions::Spec> exit;
};

} // namespace pathfinder
