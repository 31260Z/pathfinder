# Pathfinder

A movement library for VEX V5 robots running PROS — pose tracking,
calibration, controllers, and autonomous primitives, designed to ship as a
`pros conduct`-installable template. Same family as LemLib / EZ-Template,
more ambitious in scope (drift-aware control, multi-tier localization,
auto-calibration, declarative sensor configuration).

**Status (v0.1, 2026-04-25):** feature-complete per the spec. Host tests
pass (45/45 doctest targets); PROS-side adapters compile under a PROS
toolchain; quick-start `main.cpp` ships in the template.

## Documentation

- [`docs/tutorial.md`](docs/tutorial.md) — 10-minute getting started + per-feature walkthroughs.
- [`docs/design/SPEC.md`](docs/design/SPEC.md) — authoritative design (~890 lines, 17 sections + 3 appendices).
- [`pros_template/README.md`](pros_template/README.md) — how the PROS template archive is built and installed.
- [`CLAUDE.md`](CLAUDE.md) — repo conventions for contributors.

## Build (host, for unit tests)

The library is host-buildable so the math is testable off-robot. The PROS
adapter headers under `include/pathfinder/pros/` are the only files that
require a PROS toolchain; they are intentionally NOT pulled in by the
host build.

```sh
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

Compiler flags (host): `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wsign-conversion -Werror`. Library code must compile clean.

## Install on a V5 brain (PROS template)

```sh
# In your PROS project directory:
curl -LO https://github.com/<org>/pathfinder/releases/download/v0.1.0/pathfinder@0.1.0.zip
pros conduct fetch ./pathfinder@0.1.0.zip
pros conduct apply pathfinder
pros mu        # build + upload
```

To build the archive locally instead of pulling from GitHub:

```sh
bash pros_template/package.sh
```

The script copies headers into `pros_template/include/pathfinder/`, zips
the staging dir into `pathfinder@<version>.zip` in the repo root, and prints
the install commands. See [`pros_template/README.md`](pros_template/README.md).

## What's in v0.1

- **Geometry**: `Pose2`, `Vector2`, `Bot`, `NamedPoint`, `Matrix3` / `Matrix6`.
- **Sensors** (declarative): `TrackingWheel`, `Imu`, `DistanceSensor`, `LandmarkSensorConfig` with builder API + PROS adapters.
- **Calibration**: `WheelFinder`, `AutoPid`, `DriftCoeff`.
- **Controllers**: `Pid`, `ExitConditions`, `MotionProfile` (Trapezoidal + S-curve), `SlewLimiter`, `FeedForward`.
- **Odometry**: dead reckoning (default), full EKF, MCL with field-map.
- **Driving**: `TankDrive`, `ArcadeDrive`, `CurvatureDrive`, `FlightStyle::Rate` (driver-control modes); `MotorGroup` aggregate.
- **Autonomous**: `moveTo`, `moveToPose` (Boomerang), `turnTo`, `swingTo`, `follow` (Catmull-Rom + pure pursuit) with waypoint annotations (speed cap, heading hint, on-arrive callback).
- **Telemetry**: `PFTLM v1` line stream with a configurable sink.
- **PROS adapters**: `Motor`, `Rotation`, `Imu`, `Distance`, `Gps`, `Controller`.

## Deferred to v0.2

- **Async motion** (`MoveOpts{ .async = true }`). The chassis throws a clear
  runtime error today. v0.2 will add a runtime task scheduler with
  `pros::Task` so async / `waitUntil` / `cancelMotion` work under the
  hood. Use synchronous motion only in v0.1.
- **Compiled static library** (`firmware/libpathfinder.a`). v0.1 ships
  header-only — compile cost is negligible in practice. v0.2 revisits if
  upload size matters.
- **Telemetry viewer** (Python live plotter). Cut from v0.1; `PFTLM v1` is
  a stable, grep-friendly format you can pipe to whatever you like.
- **Active drift exploitation** (deliberate over-rotation for cornering
  speed). The drift-aware regulator compensates passively today; active
  exploitation is v2 per the spec.
- **AprilTag adapter** — pending VEX SDK release.
- **Holonomic / mecanum / X / swerve drivetrains** — v2 (per CLAUDE.md
  "locked decisions").
