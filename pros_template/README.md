# Pathfinder PROS template (staging area)

This directory is the staging area for the `pathfinder@<version>.zip` archive
that gets installed into a user's PROS project via `pros conduct apply
pathfinder`.

The directory layout intentionally mirrors what PROS expects inside the
template archive:

```
pros_template/
├── template.pros        manifest (name, version, system_files, user_files)
├── include/pathfinder/  copied from <repo>/include/pathfinder/ at package time
├── src/main.cpp         shipped as a USER FILE — preserved across template updates
├── firmware/            empty in v0.1 (header-only library); v0.2 will drop libpathfinder.a here
└── README.md            this file
```

## Building the template archive

Run the bundled script from the repo root:

```sh
bash pros_template/package.sh
```

The script:
1. Copies `<repo>/include/pathfinder/` into `pros_template/include/pathfinder/`
   (so the headers ship inside the archive).
2. Skips the static-lib build (Pathfinder v0.1 is header-only). v0.2 will
   compile and drop `firmware/libpathfinder.a` here.
3. Builds `pathfinder@<version>.zip` in the repo root.
4. Prints the `pros conduct fetch` / `apply` commands the user will run.

## Installing into a PROS project

After building the archive:

```sh
cd /path/to/your/pros_project
pros conduct fetch /path/to/pathfinder@0.1.0.zip
pros conduct apply pathfinder@0.1.0
pros mu        # build + upload
```

If you upgrade Pathfinder later (e.g. v0.2), repeat with the new archive:

```sh
pros conduct fetch /path/to/pathfinder@0.2.0.zip
pros conduct apply pathfinder@0.2.0
```

System files (the headers under `include/pathfinder/`) overwrite. User files
(your `src/main.cpp`) are preserved.

## What's in v0.1

- All odometry tiers: dead-reckoning, EKF, MCL with field map.
- All autonomous primitives: moveTo, moveToPose (Boomerang), turnTo, swingTo, follow (pure pursuit + Catmull-Rom spline).
- All driver-control modes: TankDrive, ArcadeDrive, CurvatureDrive, FlightStyle::Rate.
- All calibration utilities: WheelFinder, AutoPid, DriftCoeff.
- PROS adapters for: Motor, Rotation Sensor, IMU, Distance Sensor, GPS, Controller.
- Telemetry stream (PFTLM v1 line format) with a configurable sink.

## What's deferred to v0.2

- Async motion (`MoveOpts{ .async = true }`). Setting `async = true` currently
  throws a clear runtime error — the user is told v0.2 will add the runtime
  layer. The chassis runs every motion verb synchronously today; this is the
  safer default and matches LemLib's UX.
- Compiled static library (`libpathfinder.a`). The header-only build has
  near-zero compile cost in practice (it's all small templates), so v0.1
  ships header-only and revisits in v0.2 if upload size becomes an issue.
- Telemetry viewer Python script. The `PFTLM v1` line format is stable +
  grep-friendly; users can wire it to whatever live plotter they like.
