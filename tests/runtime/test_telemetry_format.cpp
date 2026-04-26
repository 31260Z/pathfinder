#include <doctest/doctest.h>

#include <pathfinder/runtime/telemetry.hpp>

#include <string>

using namespace pathfinder;

TEST_CASE("Telemetry::format_record produces a parseable PFTLM v1 line (no target)") {
    TelemetryRecord rec{};
    rec.t_ms                = 12345;
    rec.pose                = Pose2{48.12, 23.87, Angle::degrees(89.5)};
    rec.v_body              = BodyVelocity{40.2, 1.1, 5.3};
    rec.pose_cov            = Matrix3::zero();
    rec.verb                = "moveTo";
    rec.has_target          = false;
    rec.along_track_err_in  = 0.13;
    rec.cross_track_err_in  = -0.01;
    rec.left_mv             = 8400.0;
    rec.right_mv            = 7900.0;

    const std::string line = Telemetry::format_record(rec);
    CHECK(line.find("PFTLM v1") == 0);
    CHECK(line.find("t=12345")               != std::string::npos);
    CHECK(line.find("pose=(48.12,23.87,89.50)") != std::string::npos);
    CHECK(line.find("v_body=(40.20,1.10,5.30)") != std::string::npos);
    CHECK(line.find("ctrl=moveTo")           != std::string::npos);
    CHECK(line.find("along=0.13")            != std::string::npos);
    CHECK(line.find("cross=-0.01")           != std::string::npos);
    CHECK(line.find("left_mv=8400.0")        != std::string::npos);
    CHECK(line.find("right_mv=7900.0")       != std::string::npos);
    CHECK(line.find("target=")               == std::string::npos);
}

TEST_CASE("Telemetry::format_record includes target when has_target") {
    TelemetryRecord rec{};
    rec.t_ms              = 1000;
    rec.pose              = Pose2{12.0, 0.0, Angle::degrees(0.0)};
    rec.verb              = "moveToPose";
    rec.has_target        = true;
    rec.target_x_in       = 48.0;
    rec.target_y_in       = 24.0;
    rec.target_heading_deg = 90.0;

    const std::string line = Telemetry::format_record(rec);
    CHECK(line.find("ctrl=moveToPose")              != std::string::npos);
    CHECK(line.find("target=(48.00,24.00,90.00)")   != std::string::npos);
}

TEST_CASE("Telemetry: sink callback receives every emitted record") {
    Telemetry tlm;
    std::vector<std::string> captured;
    tlm.set_sink([&](std::string_view s) { captured.emplace_back(s); });

    TelemetryRecord r{};
    r.verb = "test";
    tlm.tick(r);
    tlm.tick(r);
    tlm.tick(r);

    REQUIRE(captured.size() == 3);
    for (const auto& l : captured) {
        CHECK(l.find("PFTLM v1") == 0);
        CHECK(l.find("ctrl=test") != std::string::npos);
    }
}

TEST_CASE("Telemetry: set_enabled(false) suppresses sink calls but tick still returns the line") {
    Telemetry tlm;
    int       hits = 0;
    tlm.set_sink([&](std::string_view) { ++hits; });

    tlm.set_enabled(false);
    const auto line = tlm.tick(TelemetryRecord{});
    CHECK(line.find("PFTLM v1") == 0);
    CHECK(hits == 0);

    tlm.set_enabled(true);
    tlm.tick(TelemetryRecord{});
    CHECK(hits == 1);
}

TEST_CASE("Telemetry: cov_trace formatting") {
    TelemetryRecord rec{};
    rec.pose_cov.m[0][0] = 0.1;
    rec.pose_cov.m[1][1] = 0.2;
    rec.pose_cov.m[2][2] = 0.3;
    const std::string line = Telemetry::format_record(rec);
    // 0.1+0.2+0.3 = 0.6 (allow either 0.6000 exactly or close — snprintf "%.4f"
    // gives "0.6000" since the sum is representable to 4 decimals after
    // rounding).
    CHECK(line.find("cov_trace=0.6000") != std::string::npos);
}
