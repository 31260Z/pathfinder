#pragma once

#include <pathfinder/geometry/matrix3.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/dead_reckoning.hpp>

#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <string_view>

// Hardware-agnostic telemetry record + sink (spec §14, build-order step 20).
//
// A `Telemetry` instance owns:
//   • a sink callback that receives a single rendered text line per record;
//   • a record-format helper that produces the line.
//
// The line format is a single space-separated key=value tuple prefixed with
// `PFTLM v1`; values that are tuples use parentheses-and-commas. The format
// is deliberately stable + grep-friendly so users can pipe `pros terminal`
// through awk / a Python plotter.
//
//   PFTLM v1 t=12345 pose=(48.12,23.87,89.50) v_body=(40.21,1.10,5.30)
//          ctrl=moveToPose target=(48.00,24.00,90.00) along=0.13 cross=-0.01
//          left_mv=8400.0 right_mv=7900.0
//
// The chassis owns a `Telemetry` instance and ticks it from the same path
// that drives the motors — so a record fires every controller cycle (10 ms by
// default). Users wire `set_sink(...)` to whatever they like:
// `pros::printf`, brain-LCD draw routines, file IO, etc. The default sink is
// a no-op so telemetry has zero cost when unconfigured.

namespace pathfinder {

struct TelemetryRecord {
    // Monotonic timestamp (milliseconds since the chassis was constructed,
    // host-side; on the V5 we recommend the user replace with millis() but
    // the format doesn't care which clock provides the value).
    long long t_ms = 0;

    Pose2          pose{};
    BodyVelocity   v_body{};
    Matrix3        pose_cov{};

    // The active motion verb's short name ("moveTo", "moveToPose", "turnTo",
    // "follow", "swingTo", "opcontrol", or "" when idle). The chassis sets
    // this before each tick.
    std::string_view verb = "idle";

    // Optional target descriptor — populated by motion verbs that have a
    // single end target. `has_target == false` means the verb is path-mode
    // (follow) or position-less (opcontrol).
    bool   has_target  = false;
    double target_x_in = 0.0;
    double target_y_in = 0.0;
    double target_heading_deg = 0.0;

    // Along-track / cross-track instantaneous errors as reported by the
    // active controller, when meaningful. Default 0 / 0 when the verb doesn't
    // expose them (TurnTo, opcontrol).
    double along_track_err_in = 0.0;
    double cross_track_err_in = 0.0;

    // Per-side commanded motor voltages (mV).
    double left_mv  = 0.0;
    double right_mv = 0.0;
};

class Telemetry {
public:
    using Sink = std::function<void(std::string_view)>;

    Telemetry() : start_(std::chrono::steady_clock::now()) {}

    // Replace the sink. Defaults to a no-op (`telemetry off`).
    void set_sink(Sink s) { sink_ = std::move(s); }

    // Toggle without touching the sink. Useful when the sink is already wired
    // but the user wants to silence the stream temporarily.
    void set_enabled(bool on) { enabled_ = on; }
    bool enabled() const { return enabled_; }

    // Returns "ms since this Telemetry was constructed". Available to callers
    // that want to stamp their own records — chassis uses it internally.
    long long now_ms() const {
        const auto dt = std::chrono::steady_clock::now() - start_;
        return std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
    }

    // Render-and-emit a record. Returns the rendered line for tests that
    // want to assert on it without going through the sink.
    std::string tick(TelemetryRecord rec) {
        if (rec.t_ms == 0) rec.t_ms = now_ms();
        const std::string line = format_record(rec);
        if (enabled_ && sink_) sink_(line);
        return line;
    }

    // Stand-alone formatter; exposed for tests + for users who want to
    // post-process records before forwarding to their own sink.
    static std::string format_record(const TelemetryRecord& r) {
        // ~280 chars max for the longest record (with target + path-mode +
        // long verb names). 512 leaves comfortable headroom.
        char buf[512];
        int n = 0;

        if (r.has_target) {
            n = std::snprintf(
                buf, sizeof(buf),
                "PFTLM v1 t=%lld pose=(%.2f,%.2f,%.2f) v_body=(%.2f,%.2f,%.2f) "
                "cov_trace=%.4f ctrl=%.*s target=(%.2f,%.2f,%.2f) "
                "along=%.2f cross=%.2f left_mv=%.1f right_mv=%.1f",
                r.t_ms,
                r.pose.x, r.pose.y, r.pose.heading.degrees(),
                r.v_body.v_x_ips, r.v_body.v_y_ips, r.v_body.omega_dps,
                cov_trace(r.pose_cov),
                static_cast<int>(r.verb.size()), r.verb.data(),
                r.target_x_in, r.target_y_in, r.target_heading_deg,
                r.along_track_err_in, r.cross_track_err_in,
                r.left_mv, r.right_mv);
        } else {
            n = std::snprintf(
                buf, sizeof(buf),
                "PFTLM v1 t=%lld pose=(%.2f,%.2f,%.2f) v_body=(%.2f,%.2f,%.2f) "
                "cov_trace=%.4f ctrl=%.*s "
                "along=%.2f cross=%.2f left_mv=%.1f right_mv=%.1f",
                r.t_ms,
                r.pose.x, r.pose.y, r.pose.heading.degrees(),
                r.v_body.v_x_ips, r.v_body.v_y_ips, r.v_body.omega_dps,
                cov_trace(r.pose_cov),
                static_cast<int>(r.verb.size()), r.verb.data(),
                r.along_track_err_in, r.cross_track_err_in,
                r.left_mv, r.right_mv);
        }
        if (n < 0) return std::string{};
        return std::string(buf, static_cast<std::size_t>(n));
    }

private:
    static double cov_trace(const Matrix3& mat) {
        // Matrix3 stores nine elements; trace = m00 + m11 + m22.
        return mat.m[0][0] + mat.m[1][1] + mat.m[2][2];
    }

    std::chrono::steady_clock::time_point start_;
    Sink sink_;
    bool enabled_ = true;
};

} // namespace pathfinder
