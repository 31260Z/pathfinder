#pragma once

// Umbrella header for the calibration module. Per spec §15 / §7, this
// directory hosts one-shot setup utilities that derive what's hard to
// measure with a tape: tracking-wheel offsets (Wave E), PID gains (Wave E),
// and lateral friction coefficient (Wave D).

#include <pathfinder/calibration/auto_pid.hpp>
#include <pathfinder/calibration/drift_coeff.hpp>
#include <pathfinder/calibration/wheel_finder.hpp>
