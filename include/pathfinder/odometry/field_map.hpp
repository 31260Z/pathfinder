#pragma once

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/geometry/vector2.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace pathfinder {

// Static occupancy-grid field map. Spec §8 ("Tier 3 — MCL", "Field map
// format"): a 144"×144" grid at 1" resolution, holding only the field
// perimeter and fixed obstacles (mobile game pieces are excluded). Used by
// MCL to predict expected distance-sensor readings via ray-cast.
//
// All coordinates the user supplies are in WORLD frame inches. Internally the
// map stores a row-major boolean occupancy grid. Cell (cx, cy) covers the
// world rectangle:
//   x ∈ [origin.x + cx*res, origin.x + (cx+1)*res)
//   y ∈ [origin.y + cy*res, origin.y + (cy+1)*res)
//
// Ray-cast uses an Amanatides-Woo-style DDA traversal (close cousin of
// Bresenham) — exact, gridline-snapping, and handles axis-aligned rays
// without divide-by-zero. Returns the distance from the ray origin to the
// CENTER of the first occupied cell hit, capped at `max_range_in + 1.0`
// when no obstacle is encountered (the spec contract for "out of range").
class FieldMap {
public:
    struct Config {
        double  size_x_in     = 144.0;
        double  size_y_in     = 144.0;
        double  resolution_in = 1.0;
        Vector2 origin_world_xy = {-72.0, -72.0};
    };

    struct Landmark {
        std::string name;
        Pose2       field_pose;
    };

    FieldMap() : FieldMap(Config{}) {}

    explicit FieldMap(Config cfg)
        : config_(cfg),
          cells_x_(0),
          cells_y_(0) {
        if (cfg.resolution_in <= 0.0) {
            throw std::runtime_error("FieldMap: resolution_in must be > 0");
        }
        if (cfg.size_x_in <= 0.0 || cfg.size_y_in <= 0.0) {
            throw std::runtime_error("FieldMap: size_x_in/size_y_in must be > 0");
        }
        cells_x_ = static_cast<std::size_t>(std::max(1.0, std::ceil(cfg.size_x_in / cfg.resolution_in)));
        cells_y_ = static_cast<std::size_t>(std::max(1.0, std::ceil(cfg.size_y_in / cfg.resolution_in)));
        occupancy_.assign(cells_x_ * cells_y_, false);
    }

    // ── Obstacle insertion (world coords, inches) ───────────────────────
    void add_rect_obstacle(double x_min, double y_min, double w, double h) {
        if (w <= 0.0 || h <= 0.0) return;
        const double x_max = x_min + w;
        const double y_max = y_min + h;
        // Mark every cell whose AABB overlaps the rectangle. (Cell-center
        // semantics would miss thin walls — perimeter rects are 0.5" thick
        // on a 1" grid, so a strict center-inside test never fires.) This
        // is intentionally inclusive: any cell that the obstacle touches
        // becomes "occupied" for the MCL likelihood model.
        const auto [cx0, cx1] = clip_cell_range_x(x_min, x_max);
        const auto [cy0, cy1] = clip_cell_range_y(y_min, y_max);
        const double res = config_.resolution_in;
        for (std::size_t cy = cy0; cy < cy1; ++cy) {
            const double cy_min = config_.origin_world_xy.y + static_cast<double>(cy) * res;
            const double cy_max = cy_min + res;
            if (cy_max <= y_min || cy_min >= y_max) continue;
            for (std::size_t cx = cx0; cx < cx1; ++cx) {
                const double cx_min = config_.origin_world_xy.x + static_cast<double>(cx) * res;
                const double cx_max = cx_min + res;
                if (cx_max <= x_min || cx_min >= x_max) continue;
                occupancy_[index(cx, cy)] = true;
            }
        }
    }

    void add_polygon_obstacle(const std::vector<Vector2>& vertices) {
        if (vertices.size() < 3) return;
        // AABB clip + per-cell point-in-polygon (ray-casting parity test).
        double xmin = vertices[0].x, xmax = vertices[0].x;
        double ymin = vertices[0].y, ymax = vertices[0].y;
        for (const Vector2& v : vertices) {
            xmin = std::min(xmin, v.x);
            xmax = std::max(xmax, v.x);
            ymin = std::min(ymin, v.y);
            ymax = std::max(ymax, v.y);
        }
        const auto [cx0, cx1] = clip_cell_range_x(xmin, xmax);
        const auto [cy0, cy1] = clip_cell_range_y(ymin, ymax);
        for (std::size_t cy = cy0; cy < cy1; ++cy) {
            for (std::size_t cx = cx0; cx < cx1; ++cx) {
                if (point_in_polygon(cell_center_world(cx, cy), vertices)) {
                    occupancy_[index(cx, cy)] = true;
                }
            }
        }
    }

    void add_circle_obstacle(Vector2 center, double radius) {
        if (radius <= 0.0) return;
        const double r2 = radius * radius;
        const auto [cx0, cx1] = clip_cell_range_x(center.x - radius, center.x + radius);
        const auto [cy0, cy1] = clip_cell_range_y(center.y - radius, center.y + radius);
        for (std::size_t cy = cy0; cy < cy1; ++cy) {
            for (std::size_t cx = cx0; cx < cx1; ++cx) {
                const Vector2 c = cell_center_world(cx, cy);
                const double dx = c.x - center.x;
                const double dy = c.y - center.y;
                if (dx * dx + dy * dy <= r2) {
                    occupancy_[index(cx, cy)] = true;
                }
            }
        }
    }

    // ── Queries ──────────────────────────────────────────────────────────
    bool is_occupied(Vector2 world_xy) const {
        std::size_t cx = 0, cy = 0;
        if (!world_to_cell(world_xy, cx, cy)) return false;
        return occupancy_[index(cx, cy)];
    }

    // Ray-cast from `origin_world` along `direction_rad` (world frame, +X→+Y
    // CW per Pathfinder convention; same as the bot heading). Returns the
    // distance to the center of the first occupied cell, or
    // `max_range_in + 1.0` when no hit is encountered within range or when
    // the ray starts outside the grid.
    //
    // Implementation: Amanatides & Woo's "A Fast Voxel Traversal Algorithm"
    // adapted to 2D. For each axis, compute the parametric `t` to the next
    // gridline; step to whichever axis hits first. Distance returned is
    // measured to the cell center (close enough for the MCL likelihood).
    double cast_ray(Vector2 origin_world,
                    double  direction_rad,
                    double  max_range_in) const {
        const double miss_value = max_range_in + 1.0;

        std::size_t cx = 0, cy = 0;
        const bool inside = world_to_cell(origin_world, cx, cy);
        if (!inside) return miss_value;

        // If we already start inside an obstacle, treat it as a zero-distance
        // hit. (Useful for callers — and arguably "correct" — but most MCL
        // code skips particles inside obstacles before this call.)
        if (occupancy_[index(cx, cy)]) return 0.0;

        const double res = config_.resolution_in;
        const double dx  = std::cos(direction_rad);
        const double dy  = std::sin(direction_rad);

        const int step_x = (dx > 0.0) ? 1 : (dx < 0.0 ? -1 : 0);
        const int step_y = (dy > 0.0) ? 1 : (dy < 0.0 ? -1 : 0);

        // tMax: parametric distance from origin to the next gridline along
        // each axis. tDelta: parametric distance per cell crossing.
        constexpr double k_inf = std::numeric_limits<double>::infinity();
        const double cell_origin_x = config_.origin_world_xy.x + static_cast<double>(cx) * res;
        const double cell_origin_y = config_.origin_world_xy.y + static_cast<double>(cy) * res;

        double t_max_x = k_inf;
        if (step_x > 0)      t_max_x = (cell_origin_x + res - origin_world.x) / dx;
        else if (step_x < 0) t_max_x = (cell_origin_x - origin_world.x) / dx;

        double t_max_y = k_inf;
        if (step_y > 0)      t_max_y = (cell_origin_y + res - origin_world.y) / dy;
        else if (step_y < 0) t_max_y = (cell_origin_y - origin_world.y) / dy;

        const double t_delta_x = (step_x != 0) ? std::abs(res / dx) : k_inf;
        const double t_delta_y = (step_y != 0) ? std::abs(res / dy) : k_inf;

        // Walk the grid. On every step, check the cell we just entered.
        while (true) {
            double t_hit = 0.0;
            if (t_max_x < t_max_y) {
                t_hit = t_max_x;
                cx = static_cast<std::size_t>(static_cast<long long>(cx) + step_x);
                t_max_x += t_delta_x;
            } else {
                t_hit = t_max_y;
                cy = static_cast<std::size_t>(static_cast<long long>(cy) + step_y);
                t_max_y += t_delta_y;
            }

            if (t_hit > max_range_in) return miss_value;

            // Out-of-bounds — counts as a wall hit (useful for fields where
            // the perimeter isn't explicitly added; but we still report the
            // miss_value so callers don't accidentally lock on imaginary
            // walls). Spec leaves this open; "off the field" → out of range
            // is the safest reading.
            if (cx >= cells_x_ || cy >= cells_y_) return miss_value;

            if (occupancy_[index(cx, cy)]) {
                // Distance to the center of the hit cell.
                const Vector2 ctr = cell_center_world(cx, cy);
                const double  ddx = ctr.x - origin_world.x;
                const double  ddy = ctr.y - origin_world.y;
                const double  d   = std::sqrt(ddx * ddx + ddy * ddy);
                if (d > max_range_in) return miss_value;
                return d;
            }
        }
    }

    // ── Landmarks (spec §8 "landmarks: []" section, AprilTag-extensibility) ─
    void add_landmark(std::string name, Pose2 field_pose) {
        landmarks_.push_back({std::move(name), field_pose});
    }
    const std::vector<Landmark>& landmarks() const { return landmarks_; }

    Config        config()    const { return config_; }
    std::size_t   cells_x()   const { return cells_x_; }
    std::size_t   cells_y()   const { return cells_y_; }

    // ── Built-in default: a 144"×144" perimeter-only field ─────────────
    static FieldMap default_perimeter() {
        FieldMap m{};
        // Each wall is a 0.5"-thick rect at the edge of the field.
        const double s   = m.config_.size_x_in;
        const double s2  = s / 2.0;
        const double thk = 0.5;
        // South, North, West, East walls.
        m.add_rect_obstacle(-s2,        -s2,        s,   thk);            // south
        m.add_rect_obstacle(-s2,         s2 - thk,  s,   thk);            // north
        m.add_rect_obstacle(-s2,        -s2,        thk, s);              // west
        m.add_rect_obstacle( s2 - thk,  -s2,        thk, s);              // east
        return m;
    }

    // ── JSON loaders (hand-rolled, spec §8 schema) ──────────────────────
    static FieldMap load_from_json_string(std::string_view json_text) {
        Parser p(json_text);
        return p.parse_root();
    }

    static FieldMap load_from_file(const std::string& path) {
        std::ifstream in(path);
        if (!in) {
            throw std::runtime_error("FieldMap JSON: cannot open file '" + path + "'");
        }
        std::stringstream ss;
        ss << in.rdbuf();
        return load_from_json_string(ss.str());
    }

private:
    // ── Cell math ───────────────────────────────────────────────────────
    std::size_t index(std::size_t cx, std::size_t cy) const {
        return cy * cells_x_ + cx;
    }

    Vector2 cell_center_world(std::size_t cx, std::size_t cy) const {
        return {
            config_.origin_world_xy.x + (static_cast<double>(cx) + 0.5) * config_.resolution_in,
            config_.origin_world_xy.y + (static_cast<double>(cy) + 0.5) * config_.resolution_in,
        };
    }

    bool world_to_cell(Vector2 world_xy, std::size_t& cx, std::size_t& cy) const {
        const double rx = (world_xy.x - config_.origin_world_xy.x) / config_.resolution_in;
        const double ry = (world_xy.y - config_.origin_world_xy.y) / config_.resolution_in;
        if (rx < 0.0 || ry < 0.0) return false;
        const auto ix = static_cast<std::size_t>(rx);
        const auto iy = static_cast<std::size_t>(ry);
        if (ix >= cells_x_ || iy >= cells_y_) return false;
        cx = ix;
        cy = iy;
        return true;
    }

    // Returns half-open cell range [c0, c1) covering the world AABB.
    std::pair<std::size_t, std::size_t> clip_cell_range_x(double xmin, double xmax) const {
        const double res = config_.resolution_in;
        const double r0  = (xmin - config_.origin_world_xy.x) / res;
        const double r1  = (xmax - config_.origin_world_xy.x) / res;
        const long long i0 = static_cast<long long>(std::floor(r0));
        const long long i1 = static_cast<long long>(std::ceil(r1));
        const std::size_t c0 = static_cast<std::size_t>(std::max<long long>(0, i0));
        const std::size_t c1 = static_cast<std::size_t>(
            std::clamp<long long>(i1, 0, static_cast<long long>(cells_x_)));
        return {c0, c1};
    }

    std::pair<std::size_t, std::size_t> clip_cell_range_y(double ymin, double ymax) const {
        const double res = config_.resolution_in;
        const double r0  = (ymin - config_.origin_world_xy.y) / res;
        const double r1  = (ymax - config_.origin_world_xy.y) / res;
        const long long i0 = static_cast<long long>(std::floor(r0));
        const long long i1 = static_cast<long long>(std::ceil(r1));
        const std::size_t c0 = static_cast<std::size_t>(std::max<long long>(0, i0));
        const std::size_t c1 = static_cast<std::size_t>(
            std::clamp<long long>(i1, 0, static_cast<long long>(cells_y_)));
        return {c0, c1};
    }

    static bool point_in_polygon(Vector2 p, const std::vector<Vector2>& poly) {
        // Standard horizontal-ray parity test. Includes the boundary as
        // "inside" for the cells we test (centers).
        bool inside = false;
        const std::size_t n = poly.size();
        for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
            const Vector2& a = poly[i];
            const Vector2& b = poly[j];
            const bool intersect =
                ((a.y > p.y) != (b.y > p.y)) &&
                (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x);
            if (intersect) inside = !inside;
        }
        return inside;
    }

    // ── Hand-rolled JSON parser ─────────────────────────────────────────
    // Recursive-descent. Supports objects, arrays, strings (no escapes
    // beyond \" and \\), numbers (int/float/leading-minus/exponents),
    // booleans, null. Sufficient for the spec §8 field-map schema; rejects
    // anything else with a clear `std::runtime_error("FieldMap JSON: ...")`.
    class Parser {
    public:
        explicit Parser(std::string_view text) : text_(text), pos_(0) {}

        FieldMap parse_root() {
            skip_ws();
            if (peek() != '{') fail("expected '{' at root");
            FieldMap map;
            // Two-phase: first scan for size_in / origin / resolution to
            // build the FieldMap with the right config; then a second pass
            // to ingest obstacles + landmarks. Hand-rolled JSON is small
            // enough that two passes are simpler than buffering tokens.
            FieldMap::Config cfg = parse_root_config();
            map = FieldMap{cfg};
            // Reset cursor and parse obstacles / landmarks.
            pos_ = 0;
            skip_ws();
            expect('{');
            parse_object_members([&](std::string_view key) {
                if (key == "obstacles")      parse_obstacles(map);
                else if (key == "landmarks") parse_landmarks(map);
                else                          skip_value();   // already consumed
            });
            return map;
        }

    private:
        FieldMap::Config parse_root_config() {
            const std::size_t saved = pos_;
            FieldMap::Config cfg{};
            expect('{');
            parse_object_members([&](std::string_view key) {
                if (key == "size_in") {
                    auto arr = parse_number_array();
                    if (arr.size() != 2) fail("size_in must be a 2-element array");
                    cfg.size_x_in = arr[0];
                    cfg.size_y_in = arr[1];
                } else if (key == "resolution_in") {
                    cfg.resolution_in = parse_number();
                } else if (key == "origin") {
                    skip_ws();
                    if (peek() == '"') {
                        std::string s = parse_string();
                        if (s == "center") {
                            cfg.origin_world_xy = {-cfg.size_x_in / 2.0, -cfg.size_y_in / 2.0};
                        } else {
                            fail("origin string must be \"center\"");
                        }
                    } else if (peek() == '[') {
                        auto arr = parse_number_array();
                        if (arr.size() != 2) fail("origin array must be 2 elements");
                        cfg.origin_world_xy = {arr[0], arr[1]};
                    } else {
                        fail("origin must be a string or array");
                    }
                } else {
                    skip_value();
                }
            });
            // If size_in was set but origin wasn't, default to center.
            // (Already handled above when "origin": "center" — but if both
            // size_in and origin appear, the user-supplied origin wins.
            // If no origin was given, leave the default of (-72, -72).)
            pos_ = saved;
            return cfg;
        }

        // ── obstacles parsing ───────────────────────────────────────
        void parse_obstacles(FieldMap& map) {
            skip_ws();
            expect('[');
            parse_array_elements([&] {
                expect('{');
                std::string type;
                // Stage values keyed by name; pick them up after the object
                // closes so we don't depend on key order.
                std::vector<double> rect{};                  // x, y, w, h
                std::vector<std::vector<double>> polygon{};  // vertices
                Vector2 circle_center{};
                double  circle_radius = 0.0;
                parse_object_members([&](std::string_view key) {
                    if (key == "type") {
                        type = parse_string();
                    } else if (key == "x") {
                        if (rect.size() < 1) rect.resize(4, 0.0);
                        rect[0] = parse_number();
                    } else if (key == "y") {
                        if (rect.size() < 2) rect.resize(4, 0.0);
                        rect[1] = parse_number();
                    } else if (key == "w") {
                        if (rect.size() < 3) rect.resize(4, 0.0);
                        rect[2] = parse_number();
                    } else if (key == "h") {
                        if (rect.size() < 4) rect.resize(4, 0.0);
                        rect[3] = parse_number();
                    } else if (key == "vertices") {
                        skip_ws();
                        expect('[');
                        parse_array_elements([&] {
                            polygon.push_back(parse_number_array());
                        });
                    } else if (key == "center") {
                        auto arr = parse_number_array();
                        if (arr.size() != 2) fail("circle center must be 2 elements");
                        circle_center = {arr[0], arr[1]};
                    } else if (key == "radius") {
                        circle_radius = parse_number();
                    } else {
                        skip_value();
                    }
                });
                if (type == "rect") {
                    if (rect.size() < 4) fail("rect obstacle missing x/y/w/h");
                    map.add_rect_obstacle(rect[0], rect[1], rect[2], rect[3]);
                } else if (type == "polygon") {
                    std::vector<Vector2> verts;
                    verts.reserve(polygon.size());
                    for (const auto& v : polygon) {
                        if (v.size() != 2) fail("polygon vertex must be a 2-element array");
                        verts.push_back({v[0], v[1]});
                    }
                    map.add_polygon_obstacle(verts);
                } else if (type == "circle") {
                    map.add_circle_obstacle(circle_center, circle_radius);
                } else if (type.empty()) {
                    fail("obstacle missing 'type' field");
                } else {
                    fail("unknown obstacle type '" + type + "'");
                }
            });
        }

        void parse_landmarks(FieldMap& map) {
            skip_ws();
            expect('[');
            parse_array_elements([&] {
                expect('{');
                std::string name;
                double x = 0.0, y = 0.0, heading_deg = 0.0;
                bool   has_heading = false;
                parse_object_members([&](std::string_view key) {
                    if (key == "name") {
                        name = parse_string();
                    } else if (key == "x") {
                        x = parse_number();
                    } else if (key == "y") {
                        y = parse_number();
                    } else if (key == "heading_deg") {
                        heading_deg = parse_number();
                        has_heading = true;
                    } else {
                        skip_value();
                    }
                });
                map.add_landmark(std::move(name),
                                 Pose2{x, y, has_heading ? Angle::degrees(heading_deg) : Angle{}});
            });
        }

        // ── primitive parsers ───────────────────────────────────────
        // Iterates "key": value, "key": value, ...} and dispatches the
        // user's per-key callback. The callback is responsible for parsing
        // the value (calling parse_string / parse_number / etc.).
        template <typename Fn>
        void parse_object_members(Fn&& on_member) {
            skip_ws();
            if (peek() == '}') { advance(); return; }
            while (true) {
                skip_ws();
                std::string key = parse_string();
                skip_ws();
                expect(':');
                skip_ws();
                on_member(std::string_view(key));
                skip_ws();
                if (peek() == ',') { advance(); continue; }
                if (peek() == '}') { advance(); return; }
                fail("expected ',' or '}' in object");
            }
        }

        template <typename Fn>
        void parse_array_elements(Fn&& on_elem) {
            skip_ws();
            if (peek() == ']') { advance(); return; }
            while (true) {
                skip_ws();
                on_elem();
                skip_ws();
                if (peek() == ',') { advance(); continue; }
                if (peek() == ']') { advance(); return; }
                fail("expected ',' or ']' in array");
            }
        }

        std::string parse_string() {
            skip_ws();
            if (peek() != '"') fail("expected string");
            advance();
            std::string out;
            while (pos_ < text_.size()) {
                const char c = text_[pos_++];
                if (c == '"') return out;
                if (c == '\\') {
                    if (pos_ >= text_.size()) fail("bad escape");
                    const char e = text_[pos_++];
                    switch (e) {
                        case '"':  out.push_back('"');  break;
                        case '\\': out.push_back('\\'); break;
                        case '/':  out.push_back('/');  break;
                        case 'n':  out.push_back('\n'); break;
                        case 't':  out.push_back('\t'); break;
                        case 'r':  out.push_back('\r'); break;
                        default:   fail("unsupported escape");
                    }
                } else {
                    out.push_back(c);
                }
            }
            fail("unterminated string");
            return {};   // unreachable
        }

        double parse_number() {
            skip_ws();
            const std::size_t start = pos_;
            if (peek() == '+' || peek() == '-') advance();
            while (pos_ < text_.size() &&
                   (std::isdigit(static_cast<unsigned char>(text_[pos_])) ||
                    text_[pos_] == '.' || text_[pos_] == 'e' || text_[pos_] == 'E' ||
                    text_[pos_] == '+' || text_[pos_] == '-')) {
                advance();
            }
            if (pos_ == start) fail("expected number");
            try {
                return std::stod(std::string(text_.substr(start, pos_ - start)));
            } catch (const std::exception&) {
                fail("invalid number literal");
                return 0.0;   // unreachable
            }
        }

        std::vector<double> parse_number_array() {
            skip_ws();
            expect('[');
            std::vector<double> out;
            parse_array_elements([&] { out.push_back(parse_number_or_array_element()); });
            return out;
        }

        // Helper: numbers OR nested 2-element arrays (we flatten for the
        // simple "[[x,y],[x,y],...]" polygon case via the caller's
        // dispatcher; for plain number arrays this just returns the number).
        double parse_number_or_array_element() {
            skip_ws();
            if (peek() == '[') {
                // Caller really wanted a list-of-lists; punt back through.
                // We don't expect to be called here for that case — the
                // polygon path uses parse_number_array() directly per
                // vertex — so flag any nested arrays as an error.
                fail("nested array where number expected");
            }
            return parse_number();
        }

        // Skip past a JSON value of any type without saving it.
        void skip_value() {
            skip_ws();
            const char c = peek();
            if (c == '"') { (void)parse_string(); return; }
            if (c == '{') {
                advance();
                parse_object_members([&](std::string_view) { skip_value(); });
                return;
            }
            if (c == '[') {
                advance();
                parse_array_elements([&] { skip_value(); });
                return;
            }
            if (c == 't' || c == 'f') {
                // true / false
                while (pos_ < text_.size() &&
                       std::isalpha(static_cast<unsigned char>(text_[pos_]))) advance();
                return;
            }
            if (c == 'n') {
                while (pos_ < text_.size() &&
                       std::isalpha(static_cast<unsigned char>(text_[pos_]))) advance();
                return;
            }
            (void)parse_number();
        }

        char peek() const { return (pos_ < text_.size()) ? text_[pos_] : '\0'; }
        void advance()   { ++pos_; }
        void expect(char c) {
            if (peek() != c) fail(std::string("expected '") + c + "'");
            advance();
        }
        void skip_ws() {
            while (pos_ < text_.size()) {
                const unsigned char c = static_cast<unsigned char>(text_[pos_]);
                if (std::isspace(c)) { ++pos_; }
                else if (c == '/') {
                    // Tolerate // line comments + /* block comments. Not in
                    // strict JSON, but useful for hand-edited maps.
                    if (pos_ + 1 < text_.size() && text_[pos_ + 1] == '/') {
                        while (pos_ < text_.size() && text_[pos_] != '\n') ++pos_;
                    } else if (pos_ + 1 < text_.size() && text_[pos_ + 1] == '*') {
                        pos_ += 2;
                        while (pos_ + 1 < text_.size() &&
                               !(text_[pos_] == '*' && text_[pos_ + 1] == '/')) ++pos_;
                        if (pos_ + 1 < text_.size()) pos_ += 2;
                    } else { return; }
                } else { return; }
            }
        }

        [[noreturn]] void fail(const std::string& msg) {
            throw std::runtime_error(std::string("FieldMap JSON: ") + msg);
        }

        std::string_view text_;
        std::size_t      pos_;
    };

    Config                config_;
    std::size_t           cells_x_;
    std::size_t           cells_y_;
    std::vector<bool>     occupancy_;
    std::vector<Landmark> landmarks_;
};

} // namespace pathfinder
