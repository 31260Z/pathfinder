#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

#include <stdexcept>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

namespace {
Chassis make_chassis(SimRig& rig) {
    return Chassis(make_left_group(rig),
                   make_right_group(rig),
                   make_18x18_bot(),
                   make_sensors_from_rig(rig),
                   Localization::DeadReckoning,
                   make_drive(rig));
}
} // namespace

TEST_CASE("Chassis: moveTo with .async = true throws the documented error") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.moveTo({24.0, 0.0}, MoveOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: moveToPose with .async = true throws") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.moveToPose(Pose2{24, 0, Angle::degrees(90)},
                                       MoveOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: turnTo (heading) with .async = true throws") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.turnTo(90.0, TurnOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: turnTo (point) with .async = true throws") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.turnTo(Vector2{24, 0}, TurnOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: swingTo (heading) with .async = true throws") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.swingTo(90.0, Side::Left, TurnOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: follow with .async = true throws") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    std::vector<Vector2> path{{0, 0}, {12, 0}};
    CHECK_THROWS_AS(chassis.follow(path, FollowOpts{.async = true}),
                    std::runtime_error);
}

TEST_CASE("Chassis: waitUntil and waitUntilDone throw the runtime-not-installed error") {
    SimRig rig;
    Chassis chassis = make_chassis(rig);
    CHECK_THROWS_AS(chassis.waitUntil(12.0),    std::runtime_error);
    CHECK_THROWS_AS(chassis.waitUntilDone(),    std::runtime_error);
}
