# Pathfinder

Movement library for VEX V5 robots running PROS. Pose tracking, calibration, controllers, and autonomous primitives — designed to ship as a `pros conduct`-installable template.

- **Design spec:** [`docs/design/SPEC.md`](docs/design/SPEC.md)
- **Working notes / conventions:** [`CLAUDE.md`](CLAUDE.md)

## Build (host, for unit tests)

The library is currently host-buildable (no V5 brain required) so we can run unit tests on the math.

```sh
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

## Build & flash (V5 brain, eventual)

Pathfinder will ship as a PROS template:

```sh
pros conduct fetch ./pathfinder@<version>.zip
pros conduct apply pathfinder
pros mu        # build + upload to brain
```

See `docs/design/SPEC.md` §14 for the full install path and quick-start `main.cpp`.
