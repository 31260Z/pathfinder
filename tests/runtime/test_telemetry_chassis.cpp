#include <doctest/doctest.h>

#include "../chassis/sim_helpers.hpp"

#include <pathfinder/chassis/chassis.hpp>
#include <pathfinder/runtime/telemetry.hpp>

#include <regex>
#include <string>
#include <vector>

using namespace pathfinder;
using pathfinder_test::SimRig;
using pathfinder_test::make_18x18_bot;
using pathfinder_test::make_drive;
using pathfinder_test::make_left_group;
using pathfinder_test::make_right_group;
using pathfinder_test::make_sensors_from_rig;

TEST_CASE("Chassis: telemetry sink emits records during a moveTo and lines parse") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    std::vector<std::string> lines;
    chassis.set_telemetry_sink([&](std::string_view s) { lines.emplace_back(s); });

    // Run a moveTo with a tight timeout so we don't sleep forever. SimBot
    // isn't wired into this test (the chassis won't actually advance pose),
    // but the moveTo loop still ticks once and emits at least one record
    // before bailing.
    MoveOpts o{};
    o.timeout_ms = 5.0;   // smaller than 10ms tick so loop exits after one iteration
    chassis.moveTo({24.0, 0.0}, o);

    REQUIRE(lines.size() >= 1);
    // Loose regex: PFTLM v1 t=N pose=(X,Y,H) ...
    std::regex re(R"(^PFTLM v1 t=\d+ pose=\(-?\d+\.\d+,-?\d+\.\d+,-?\d+\.\d+\) .*ctrl=moveTo)");
    for (const auto& l : lines) {
        CHECK(std::regex_search(l, re));
    }
}

TEST_CASE("Chassis: active_verb tags each motion call") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    CHECK(chassis.active_verb() == "idle");

    std::vector<std::string> verbs;
    chassis.set_telemetry_sink([&](std::string_view s) {
        // Pull out the substring between "ctrl=" and the next space.
        const auto p = s.find("ctrl=");
        if (p == std::string_view::npos) return;
        const auto start = p + 5;
        const auto end   = s.find(' ', start);
        verbs.emplace_back(s.substr(start, (end == std::string_view::npos)
                                              ? std::string_view::npos
                                              : end - start));
    });

    MoveOpts mo{};
    mo.timeout_ms = 5.0;
    chassis.moveTo({1.0, 0.0}, mo);
    TurnOpts to{};
    to.timeout_ms = 5.0;
    chassis.turnTo(45.0, to);

    REQUIRE(verbs.size() >= 2);
    bool saw_move = false;
    bool saw_turn = false;
    for (const auto& v : verbs) {
        if (v == "moveTo") saw_move = true;
        if (v == "turnTo") saw_turn = true;
    }
    CHECK(saw_move);
    CHECK(saw_turn);

    // After all calls return, we're back to idle.
    CHECK(chassis.active_verb() == "idle");
}

TEST_CASE("Chassis: telemetry can be silenced via set_enabled(false)") {
    SimRig rig;
    Chassis chassis(make_left_group(rig),
                    make_right_group(rig),
                    make_18x18_bot(),
                    make_sensors_from_rig(rig),
                    Localization::DeadReckoning,
                    make_drive(rig));

    int hits = 0;
    chassis.set_telemetry_sink([&](std::string_view) { ++hits; });
    chassis.telemetry().set_enabled(false);

    MoveOpts o{};
    o.timeout_ms = 5.0;
    chassis.moveTo({1.0, 0.0}, o);
    CHECK(hits == 0);

    chassis.telemetry().set_enabled(true);
    chassis.moveTo({1.0, 0.0}, o);
    CHECK(hits >= 1);
}
