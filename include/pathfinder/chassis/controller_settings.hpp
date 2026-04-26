#pragma once

#include <pathfinder/controllers/feed_forward.hpp>
#include <pathfinder/controllers/motion_profile.hpp>
#include <pathfinder/controllers/pid.hpp>
#include <pathfinder/controllers/slew_limiter.hpp>

#include <limits>
#include <optional>

namespace pathfinder {

// Slew limiter doesn't ship with a `Spec` struct (its constructor takes the
// rate directly), so wrap one here for the controller-stack designated-init
// ergonomics.
struct SlewSpec {
    double max_rate_per_sec = std::numeric_limits<double>::infinity();
};

// One-axis controller stack. The chassis owns two — `lateral_` (along- and
// cross-track) and `angular_` (heading). Per-call `MoveOpts` / `TurnOpts` /
// `FollowOpts` may override the `feedback` gains and `exit` conditions; the
// rest of the stack (FF, profile, slew) is global until Wave G adds richer
// per-call overrides.
struct ControllerStack {
    Pid::Gains  feedback{1.0, 0.0, 0.1};
    FeedForward ff{0.15, 0.02, 0.0};

    // No motion profile by default — drop-in PID-only matches LemLib-style
    // behaviour. Users opt in by setting one of these (mutually exclusive in
    // practice; if both populated, Trapezoidal wins).
    std::optional<motion_profile::Trapezoidal> profile;
    std::optional<motion_profile::SCurve>      s_profile;

    SlewSpec slew{};
};

enum class ControllerAxis { Lateral, Angular };

} // namespace pathfinder
