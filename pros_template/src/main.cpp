// Pathfinder v0.1 — quick-start main.cpp.
//
// This file is shipped as a TEMPLATE USER FILE: it lands in your project the
// first time you `pros conduct apply pathfinder`, but is preserved across
// template updates so you don't lose local edits. Customize freely.
//
// For the full API, see <pathfinder/pathfinder.hpp>; for a guided tour, see
// the tutorial at docs/tutorial.md in the Pathfinder repo.

#include "main.h"

#include <pathfinder/pathfinder.hpp>
#include <pathfinder/pros/all.hpp>

using namespace pathfinder;
using namespace pathfinder::pros_v5;

// ── Hardware ─────────────────────────────────────────────────────────
// Edit these to match your robot. The right side is reversed because the
// motors face the opposite direction relative to the cartridge.
auto left_motors  = motor_group({1, 2, 3});
auto right_motors = motor_group({4, 5, 6}, /*reversed=*/true);

// ── Bot footprint ────────────────────────────────────────────────────
// 18×18 inches (a standard VRC footprint), origin at the back-left corner.
// Add named points later, e.g. .point("intake", forward=5, right=10).
auto bot = Bot()
    .footprint(18.0, 18.0)
    .origin(Corner::BackLeft);

// ── Sensors (declarative) ─────────────────────────────────────────────
// One IMU on port 7, mounted Z-up (the chip's printing faces upward) with
// its body-X axis aligned with the bot's forward direction. Offsets are
// from the back-left bot corner to the IMU's center.
auto sensors = Sensors()
    .add(Imu{
        .sensor       = imu(7),
        .mounting     = ImuMounting::ZUp_XForward,
        .offset_xy_in = {9.0, 9.0},
        .offset_z_in  = 4.0,
    });

// ── Chassis ──────────────────────────────────────────────────────────
// Tier-1 dead reckoning. Upgrade to Localization::Ekf or
// Localization::Mcl{...} once you have multiple IMUs / distance sensors / a
// loaded field map.
Chassis chassis(left_motors, right_motors, bot, sensors,
                Localization::DeadReckoning,
                Drive{
                    .track_width_in               = 12.0,
                    .wheel_diameter_in            = 3.25,
                    .gear_ratio                   = 36.0 / 84.0,
                    .lateral_friction_coefficient = 1.0,   // omni; raise for traction
                });

// ── PROS entry points ─────────────────────────────────────────────────

void initialize() {
    chassis.calibrate();   // blocks ~2s while the IMU completes its self-cal

    // Plumb telemetry to the PROS console. Anything you `pros terminal` will
    // see PFTLM v1 lines streaming at the controller cadence (~100 Hz).
    chassis.set_telemetry_sink([](std::string_view line) {
        pros::printf("%.*s\n", static_cast<int>(line.size()), line.data());
    });
}

void disabled()              {}
void competition_initialize() {}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    auto poll = make_axis_reader(controller);
    chassis.opcontrol(FlightStyle::Rate{
                          .max_forward_ips = 60.0,
                          .max_yaw_dps     = 270.0,
                      },
                      poll,
                      []{ return pros::competition::is_disabled(); });
}

void autonomous() {
    chassis.setPose(0, 0, 0);     // declare current pose as field origin, +X forward
    chassis.moveTo({24, 0});      // drive 24" forward
    chassis.turnTo(90);           // turn to face +Y
    chassis.moveTo({24, 24});     // drive 24" left
}
