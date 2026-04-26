#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/geometry/pose2.hpp>
#include <pathfinder/odometry/field_map.hpp>
#include <pathfinder/odometry/mcl.hpp>
#include <pathfinder/sensors/distance_sensor.hpp>
#include <pathfinder/sensors/landmark_sensor_config.hpp>
#include <pathfinder/sensors/mocks.hpp>

#include <cmath>
#include <memory>
#include <vector>

using namespace pathfinder;

namespace {

// Helper: build an MCL config with a known field map and reasonable defaults.
Mcl::Config make_default_config() {
    Mcl::Config c{};
    c.field_map = FieldMap{};   // empty 144x144
    // Add a wall on the east side at x=50 so ray-cast tests have something to
    // hit.
    c.field_map.add_rect_obstacle(50.0, -72.0, 1.0, 144.0);
    c.particle_count = 200;
    c.process_sigma_xy_in       = 0.01;
    c.process_sigma_heading_rad = 0.001;
    c.distance_sigma_in         = 1.0;
    c.landmark_sigma_xy_in      = 0.5;
    c.landmark_sigma_heading_rad= 0.05;
    c.rng_seed                  = 0xDEADBEEFULL;
    return c;
}

DistanceSensor make_distance_sensor(std::shared_ptr<FakeDistance> fake,
                                    Vector2 offset = {0, 0},
                                    double  theta_deg = 0.0,
                                    double  max_range_in = 80.0,
                                    double  sigma_in = 1.0) {
    DistanceSensor s{};
    s.sensor       = fake;
    s.offset_xy_in = offset;
    s.theta_deg    = theta_deg;
    s.max_range_in = max_range_in;
    s.sigma_in     = sigma_in;
    return s;
}

LandmarkSensorConfig make_landmark_sensor(std::shared_ptr<FakeLandmark> fake,
                                          Vector2 offset = {0, 0}) {
    LandmarkSensorConfig l{};
    l.sensor       = fake;
    l.offset_xy_in = offset;
    return l;
}

} // namespace

TEST_CASE("MCL: no motion + no measurements leaves pose near initial pose") {
    Mcl mcl(make_default_config(), Pose2{10.0, 5.0, Angle::degrees(45.0)},
            /*xy_sigma*/ 0.5, /*h_sigma*/ 0.02);
    const Pose2 p = mcl.pose();
    CHECK(p.x == doctest::Approx(10.0).epsilon(0.20));
    CHECK(p.y == doctest::Approx(5.0).epsilon(0.20));
    CHECK(p.heading.degrees() == doctest::Approx(45.0).epsilon(0.20));
}

TEST_CASE("MCL: predict forward translates the pose mean") {
    Mcl mcl(make_default_config(), Pose2{0.0, 0.0, Angle{}});
    // Drive 1 inch forward at heading 0.
    mcl.predict(1.0, 0.0, 0.0);
    const Pose2 p = mcl.pose();
    CHECK(p.x == doctest::Approx(1.0).epsilon(0.10));
    CHECK(p.y == doctest::Approx(0.0).epsilon(0.10));
}

TEST_CASE("MCL: many predict steps move the pose by the cumulative motion") {
    Mcl mcl(make_default_config(), Pose2{0.0, 0.0, Angle{}});
    for (int i = 0; i < 10; ++i) {
        mcl.predict(0.5, 0.0, 0.0);
    }
    const Pose2 p = mcl.pose();
    CHECK(p.x == doctest::Approx(5.0).epsilon(0.10));
    CHECK(std::abs(p.y) < 0.5);
}

TEST_CASE("MCL: predict applies bot-frame motion in particle frame (heading +π/2 → +Y)") {
    Mcl mcl(make_default_config(),
            Pose2{0.0, 0.0, Angle::radians(k_pi / 2.0)});
    // Drive 2 inches forward (in body frame). Body-X at heading +π/2 is world +Y.
    mcl.predict(2.0, 0.0, 0.0);
    const Pose2 p = mcl.pose();
    CHECK(std::abs(p.x) < 0.3);
    CHECK(p.y == doctest::Approx(2.0).epsilon(0.10));
}

TEST_CASE("MCL: distance sensor against a wall — particles converge toward truth") {
    auto cfg = make_default_config();
    cfg.distance_sigma_in = 0.5;

    // Truth: bot at (0, 0) facing +X. Wall at x=50 → distance reading ≈ 50.5.
    Mcl mcl(cfg, Pose2{}, /*xy_sigma*/ 5.0, /*h_sigma*/ 0.05);

    auto fake = std::make_shared<FakeDistance>();
    fake->set_distance(50.5);
    fake->set_valid(true);
    std::vector<DistanceSensor> sensors{ make_distance_sensor(fake) };

    // Run several update cycles; each refines the particle cloud.
    for (int i = 0; i < 8; ++i) {
        mcl.update_distance(sensors);
    }
    const Pose2 p = mcl.pose();
    // Expect convergence toward x=0 — the only x consistent with seeing the
    // wall at distance 50.5. Before updates we were spread ±5"; after, mean
    // should be much tighter.
    CHECK(std::abs(p.x) < 1.5);
}

TEST_CASE("MCL: landmark observation consistent with truth — particles converge") {
    auto cfg = make_default_config();
    cfg.particle_count             = 500;
    cfg.landmark_sigma_xy_in       = 0.3;
    cfg.landmark_sigma_heading_rad = 0.03;

    // True pose: (10, 0, 0). Landmark at (20, 0, 0) in field; in bot frame,
    // that's at relative pose (10, 0, 0).
    const Pose2 truth{10.0, 0.0, Angle{}};
    Mcl mcl(cfg, truth, /*xy_sigma*/ 4.0, /*h_sigma*/ 0.1);

    auto fake = std::make_shared<FakeLandmark>();
    LandmarkObservation obs{};
    obs.landmark_field_pose  = Pose2{20.0, 0.0, Angle{}};
    obs.relative_observation = Pose2{10.0, 0.0, Angle{}};
    fake->set_observation(obs);

    std::vector<LandmarkSensorConfig> sensors{ make_landmark_sensor(fake) };

    for (int i = 0; i < 12; ++i) {
        mcl.update_landmarks(sensors);
    }
    const Pose2 p = mcl.pose();
    CHECK(p.x == doctest::Approx(10.0).epsilon(0.30));
    CHECK(p.y == doctest::Approx(0.0).epsilon(1.0));
    CHECK(p.heading.radians() == doctest::Approx(0.0).epsilon(0.30));
}

TEST_CASE("MCL: kidnap recovery — particles redistribute toward consistent pose") {
    auto cfg = make_default_config();
    cfg.particle_count             = 500;
    cfg.landmark_sigma_xy_in       = 5.0;     // generous tolerance for kidnap
    cfg.landmark_sigma_heading_rad = 0.50;
    cfg.process_sigma_xy_in        = 0.3;     // ample noise for re-spreading
    cfg.process_sigma_heading_rad  = 0.02;

    // Initialize all particles around a wrong pose, but with enough initial
    // sigma that some particles span the field (a real "all collapsed" kidnap
    // requires injecting noise before any chance of recovery).
    const Pose2 wrong{-30.0, -30.0, Angle{}};
    Mcl mcl(cfg, wrong, /*xy_sigma*/ 2.0, /*h_sigma*/ 0.05);
    const Pose2 before = mcl.pose();

    // True pose is actually (40, 10, 0). Landmark at (50, 10, 0).
    auto fake = std::make_shared<FakeLandmark>();
    LandmarkObservation obs{};
    obs.landmark_field_pose  = Pose2{50.0, 10.0, Angle{}};
    obs.relative_observation = Pose2{10.0, 0.0, Angle{}};
    fake->set_observation(obs);

    std::vector<LandmarkSensorConfig> sensors{ make_landmark_sensor(fake) };

    // Inject process noise via predict, then re-weight. After enough rounds
    // the cloud should drift away from the original wrong pose toward the
    // consistent location.
    for (int i = 0; i < 30; ++i) {
        mcl.predict(0.0, 0.0, 0.0);   // pure noise injection
        mcl.update_landmarks(sensors);
    }

    const Pose2 after = mcl.pose();
    // Cloud should have moved meaningfully away from the wrong pose.
    const double moved_x = after.x - before.x;
    const double moved_y = after.y - before.y;
    CHECK(moved_x > 1.0);
    CHECK(moved_y > 1.0);
}

TEST_CASE("MCL: determinism — same seed produces identical particle sequence") {
    auto cfg1 = make_default_config();
    auto cfg2 = cfg1;

    Mcl a(cfg1, Pose2{1.0, 2.0, Angle::degrees(15.0)}, 0.5, 0.05);
    Mcl b(cfg2, Pose2{1.0, 2.0, Angle::degrees(15.0)}, 0.5, 0.05);

    a.predict(1.0, 0.0, 0.01);
    b.predict(1.0, 0.0, 0.01);

    REQUIRE(a.particles().size() == b.particles().size());
    for (std::size_t i = 0; i < a.particles().size(); ++i) {
        CHECK(a.particles()[i].pose.x == doctest::Approx(b.particles()[i].pose.x));
        CHECK(a.particles()[i].pose.y == doctest::Approx(b.particles()[i].pose.y));
        CHECK(a.particles()[i].pose.heading.radians()
              == doctest::Approx(b.particles()[i].pose.heading.radians()));
    }

    // Different seed → diverging samples.
    auto cfg3 = make_default_config();
    cfg3.rng_seed = 0xFEEDBEEFULL;
    Mcl c(cfg3, Pose2{1.0, 2.0, Angle::degrees(15.0)}, 0.5, 0.05);
    c.predict(1.0, 0.0, 0.01);
    bool any_diff = false;
    for (std::size_t i = 0; i < a.particles().size(); ++i) {
        if (std::abs(a.particles()[i].pose.x - c.particles()[i].pose.x) > 1e-9) {
            any_diff = true;
            break;
        }
    }
    CHECK(any_diff);
}

TEST_CASE("MCL: set_pose re-seeds the cloud at the new pose") {
    Mcl mcl(make_default_config(), Pose2{0, 0, Angle{}});
    mcl.set_pose(Pose2{42.0, -10.0, Angle::degrees(180.0)}, 0.1, 0.01);
    const Pose2 p = mcl.pose();
    CHECK(p.x == doctest::Approx(42.0).epsilon(0.05));
    CHECK(p.y == doctest::Approx(-10.0).epsilon(0.05));
    // Heading wraps; 180° and -180° are equivalent. Compare via shortest_angle.
    const double h_err = shortest_angle(Angle::degrees(180.0), p.heading).rad;
    CHECK(std::abs(h_err) < (5.0 * k_deg_to_rad));
}

TEST_CASE("MCL: pose_covariance is non-zero after a noisy initialization") {
    auto cfg = make_default_config();
    cfg.particle_count = 500;
    Mcl mcl(cfg, Pose2{0, 0, Angle{}}, 1.0, 0.1);
    const Matrix3 cov = mcl.pose_covariance();
    // Diagonal entries should be roughly σ² (1 and 0.01).
    CHECK(cov.m[0][0] > 0.5);
    CHECK(cov.m[0][0] < 2.0);
    CHECK(cov.m[1][1] > 0.5);
    CHECK(cov.m[1][1] < 2.0);
    CHECK(cov.m[2][2] > 0.0);
    CHECK(cov.m[2][2] < 0.1);
}

TEST_CASE("MCL: distance update with no fresh sensor reading is a no-op") {
    auto cfg = make_default_config();
    Mcl mcl(cfg, Pose2{0, 0, Angle{}}, 0.5, 0.01);
    auto fake = std::make_shared<FakeDistance>();
    fake->set_valid(false);
    fake->set_distance(0.0);
    std::vector<DistanceSensor> sensors{ make_distance_sensor(fake) };
    const Pose2 before = mcl.pose();
    mcl.update_distance(sensors);
    const Pose2 after = mcl.pose();
    CHECK(after.x == doctest::Approx(before.x));
    CHECK(after.y == doctest::Approx(before.y));
}

TEST_CASE("MCL: landmark update with no observation is a no-op") {
    auto cfg = make_default_config();
    Mcl mcl(cfg, Pose2{0, 0, Angle{}}, 0.5, 0.01);
    auto fake = std::make_shared<FakeLandmark>();
    // No observation set.
    std::vector<LandmarkSensorConfig> sensors{ make_landmark_sensor(fake) };
    const Pose2 before = mcl.pose();
    mcl.update_landmarks(sensors);
    const Pose2 after = mcl.pose();
    CHECK(after.x == doctest::Approx(before.x));
    CHECK(after.y == doctest::Approx(before.y));
}

TEST_CASE("MCL: predict adds rotation to particle headings") {
    auto cfg = make_default_config();
    cfg.process_sigma_heading_rad = 0.001;
    Mcl mcl(cfg, Pose2{0, 0, Angle{}}, 0.01, 0.001);
    const Pose2 before = mcl.pose();
    mcl.predict(0.0, 0.0, k_pi / 4.0);   // 45° turn
    const Pose2 after = mcl.pose();
    CHECK(after.heading.radians() - before.heading.radians()
          == doctest::Approx(k_pi / 4.0).epsilon(0.05));
}
