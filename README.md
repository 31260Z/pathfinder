# 31260Z Pathfinder

Movement library for VEX V5 robots running PROS — pose tracking, calibration,
controllers, and autonomous primitives. Same family as LemLib / EZ-Template,
more ambitious in scope (drift-aware control, multi-tier localization,
auto-calibration, declarative sensor config).

**Status (v0.1):** feature-complete. 45/45 host tests pass; PROS adapters compile
under the PROS toolchain.

## Quick start

```cpp
#include "pathfinder/pathfinder.hpp"

// 1. Build your config
auto drive = pathfinder::Drive{
    .left_motors = {
        pathfinder::MotorGroup::from_pros({-1, -2, -3})
    },
    .right_motors = {
        pathfinder::MotorGroup::from_pros({4, 5, 6})
    },
    .track_width_in = 12.5,
    .wheel_diameter = 2.75,       // default: 2.75"
    .gear_ratio = pathfinder::GearRatio::green,  // 600 rpm
    .horizontal_drift_factor = 6.0,
};

// 2. Declare sensors (builds tracking wheels + IMU internally)
auto sensors = pathfinder::SensorBuilder{}
    .with_left_motors({-1, -2, -3})   // reuse port list for tracking
    .with_right_motors({4, 5, 6})
    .with_tracking_wheels({           // optional — use motor encoders otherwise
        pathfinder::TrackingWheel{2, 3, 2.75}.offset(pathfinder::Vector2{0, 0}),
        pathfinder::TrackingWheel{4, 5, 2.75}.offset(pathfinder::Vector2{0, -7}),
    })
    .with_imu(7)
    .build();

// 3. Create the chassis
auto chassis = pathfinder::Chassis(drive, sensors);
chassis.resetPose({0, 0, 90});       // start at center facing right

// 4. Run an autonomous routine
void autonomous() {
    chassis.moveTo({24, 0});          // drive 24" forward
    chassis.turnTo(180);              // turn around
    chassis.follow({{{24, 0}}, {{48, 24}}, {{72, 0}}});  // spline path
}

// 5. Run driver control
void opcontrol() {
    double left = controller_get_analog(ANALOG_LEFT_Y);
    double right = controller_get_analog(ANALOG_RIGHT_X);
    chassis.opcontrol(pathfinder::TankDrive{left, right});
}
```

## How it works

### Module layout

```
include/pathfinder/
  geometry/         Pose2, Vector2, Matrix3, Bot, NamedPoint
  sensors/          TrackingWheel, Imu, DistanceSensor, LandmarkSensor
  calibration/      WheelFinder, AutoPid, DriftCoeff
  odometry/         DeadReckoning, EKF (Kalman filter), MCL (particle filter)
  controllers/      PID, ExitConditions, MotionProfile, SlewLimiter, FeedForward
  driving/          TankDrive, ArcadeDrive, FlightStyle::Rate
  autonomous/       moveTo, moveToPose (Boomerang), turnTo, follow (pure pursuit)
  chassis/          Chassis (god-object), Drive config
  runtime/          Telemetry stream
  pros/             PROS adapters (Motor, Imu, GPS, Distance, Rotation)
```

### Pose tracking (6-DOF)

Pathfinder tracks full state: `(x, y, heading)` + body-frame velocity `(v_x, v_y, ω)`.
Three tiers available:

| Tier | Class | When to use |
|---|---|---|
| 1 | `DeadReckoning` | Default — good enough for 99% of matches |
| 2 | `Ekf` | When you have GPS or landmark inputs |
| 3 | `Mcl` | Global localization with a field map |

Drift awareness is built in — the cross-track regulator uses `v_y` for
lookahead prediction, and motion chaining inherits lateral velocity between
segments.

### Path following

Waypoints define a Catmull-Rom spline, tracked via pure pursuit:

```cpp
chassis.follow(
    {{{{0, 0}}, {{24, 0}}, {{48, 24}}}},  // waypoints
    pathfinder::FollowOptions{
        .speed_cap_ips = 30.0,
        .lookahead_distance_in = 8.0,
    }
);
```

Each waypoint supports optional `speed_cap_ips`, `heading_deg` (soft hint), and
`on_arrive` callback.

### Driver control modes

| Mode | Input | Description |
|---|---|---|
| `TankDrive` | Left, right speeds [-1, 1] | Raw per-side |
| `ArcadeDrive` | Throttle, turn [-1, 1] | Forward + rotate |
| `FlightStyle::Rate` | Throttle, turn rate deg/s | Gyro-stabilized rate |

### Calibration (automatic)

```cpp
// Finds real tracking-wheel offsets
auto offsets = pathfinder::WheelFinder(drive, sensors).calibrate();

// Tunes PD gains for autonomous
auto settings = pathfinder::AutoPid(drive, sensors).tune({
    .target_distance_in = 24.0,
    .iterations = 3,
});
```

## Build (host tests)

```sh
cmake -B build -DTESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

## Install on V5 brain

```sh
# In your PROS project:
bash pros_template/package.sh
pros conduct fetch ./pathfinder@0.1.0.zip
pros conduct apply pathfinder
pros mu
```

## What's in v0.1

- **Geometry**: `Pose2`, `Vector2`, `Bot`, `NamedPoint`, `Matrix3` / `Matrix6`
- **Sensors**: `TrackingWheel`, `Imu`, `DistanceSensor`, `LandmarkSensorConfig` with builder API
- **Calibration**: `WheelFinder`, `AutoPid`, `DriftCoeff`
- **Controllers**: `PID`, `ExitConditions`, `MotionProfile` (trapezoidal + S-curve), `SlewLimiter`, `FeedForward`
- **Odometry**: dead reckoning, full EKF, MCL with field-map
- **Driving**: `TankDrive`, `ArcadeDrive`, `CurvatureDrive`, `FlightStyle::Rate`
- **Autonomous**: `moveTo`, `moveToPose` (Boomerang), `turnTo`, `swingTo`, `follow` (Catmull-Rom + pure pursuit)
- **Telemetry**: `PFTLM v1` line stream with configurable sink
- **PROS adapters**: `Motor`, `Rotation`, `Imu`, `Distance`, `Gps`, `Controller`

## Deferred to v0.2

- **Async motion** (`MoveOpts{ .async = true }`) — currently throws runtime error
- **Compiled static library** (`firmware/libpathfinder.a`)
- **Telemetry viewer** (Python live plotter)
- **Active drift exploitation** (deliberate over-rotation for cornering)
- **AprilTag adapter** — pending VEX SDK release
- **Holonomic / mecanum / X / swerve** — v2
