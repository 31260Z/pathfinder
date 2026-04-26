#include <doctest/doctest.h>

#include "sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

TEST_CASE("Chassis: named_point_offset returns the configured bot-frame offset") {
    SimRig rig;
    Bot bot;
    bot.footprint(18.0, 18.0)
       .origin(Corner::BackLeft)
       .point("intake", forward = 5.0, right = 10.0)
       .point("claw",   forward = 15.0, left  = 9.0);

    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    std::move(bot),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    const Vector2 intake = chassis.named_point_offset("intake");
    CHECK(intake.x == doctest::Approx(5.0));
    CHECK(intake.y == doctest::Approx(10.0));

    const Vector2 claw = chassis.named_point_offset("claw");
    CHECK(claw.x == doctest::Approx(15.0));
    CHECK(claw.y == doctest::Approx(-9.0));
}

TEST_CASE("Chassis: named_point_offset throws for unknown name") {
    SimRig rig;
    Bot bot;
    bot.footprint(18.0, 18.0).origin(Corner::BackLeft);

    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    std::move(bot),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    CHECK_THROWS_AS(chassis.named_point_offset("does_not_exist"), std::out_of_range);
}
