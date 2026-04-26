#pragma once

#include <pathfinder/geometry/vector2.hpp>
#include <pathfinder/sensors/interfaces.hpp>

#include <memory>

namespace pathfinder {

// User-facing config for a landmark-style sensor. The sensor itself
// (`ILandmark`) emits `LandmarkObservation`s; this struct just records where
// the sensor sits on the bot so the EKF/MCL can transform the relative
// observation into a chassis-pose update.
//
// Concrete `ILandmark` implementations:
//   - `VexGpsSensor`   — wraps the VEX GPS sensor (Wave G).
//   - `AprilTagSensor` — wraps the VEX AprilTag SDK (announced at Worlds
//                        2026, deferred until the SDK ships).
//
// (The struct is named `LandmarkSensorConfig` rather than `LandmarkSensor` to
// avoid a name clash with the abstract `ILandmark` interface and the spec's
// abstract-base nomenclature.)
struct LandmarkSensorConfig {
    std::shared_ptr<ILandmark> sensor;
    Vector2 offset_xy_in = {0.0, 0.0};
    double  offset_z_in  = 0.0;
};

} // namespace pathfinder
