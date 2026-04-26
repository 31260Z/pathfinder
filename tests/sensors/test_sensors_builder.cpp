#include <doctest/doctest.h>

#include <pathfinder/sensors/sensors.hpp>

#include <memory>

using namespace pathfinder;

TEST_CASE("Sensors: empty builder has no entries") {
    Sensors s;
    CHECK(s.tracking_wheels().empty());
    CHECK(s.imus().empty());
    CHECK(s.distance_sensors().empty());
    CHECK(s.landmark_sensors().empty());
}

TEST_CASE("Sensors: add returns *this for chaining") {
    Sensors s;
    Sensors& ref = s.add(TrackingWheel{});
    CHECK(&ref == &s);
}

TEST_CASE("Sensors: chained construction collects every sensor type") {
    auto rot1 = std::make_shared<FakeRotation>();
    auto rot2 = std::make_shared<FakeRotation>();
    auto imu1 = std::make_shared<FakeImu>();
    auto dist = std::make_shared<FakeDistance>();
    auto lmk  = std::make_shared<FakeLandmark>();

    auto s = Sensors()
        .add(TrackingWheel{ .sensor = rot1, .axis = Axis::X })
        .add(TrackingWheel{ .sensor = rot2, .axis = Axis::Y })
        .add(Imu{           .sensor = imu1, .mounting = ImuMounting::ZUp_XForward })
        .add(DistanceSensor{ .sensor = dist })
        .add(LandmarkSensorConfig{ .sensor = lmk });

    CHECK(s.tracking_wheels().size()  == 2);
    CHECK(s.imus().size()             == 1);
    CHECK(s.distance_sensors().size() == 1);
    CHECK(s.landmark_sensors().size() == 1);
}

TEST_CASE("Sensors: insertion order is preserved") {
    auto a = std::make_shared<FakeRotation>();
    auto b = std::make_shared<FakeRotation>();
    auto c = std::make_shared<FakeRotation>();

    auto s = Sensors()
        .add(TrackingWheel{ .sensor = a, .sigma_along_in = 0.01 })
        .add(TrackingWheel{ .sensor = b, .sigma_along_in = 0.02 })
        .add(TrackingWheel{ .sensor = c, .sigma_along_in = 0.03 });

    CHECK(s.tracking_wheels()[0].sensor.get() == a.get());
    CHECK(s.tracking_wheels()[1].sensor.get() == b.get());
    CHECK(s.tracking_wheels()[2].sensor.get() == c.get());
    CHECK(s.tracking_wheels()[0].sigma_along_in == doctest::Approx(0.01));
    CHECK(s.tracking_wheels()[1].sigma_along_in == doctest::Approx(0.02));
    CHECK(s.tracking_wheels()[2].sigma_along_in == doctest::Approx(0.03));
}

TEST_CASE("Sensors: parallel_wheels / perpendicular_wheels partition by axis") {
    auto p1 = std::make_shared<FakeRotation>();
    auto p2 = std::make_shared<FakeRotation>();
    auto q1 = std::make_shared<FakeRotation>();

    auto s = Sensors()
        .add(TrackingWheel{ .sensor = p1, .axis = Axis::X })
        .add(TrackingWheel{ .sensor = q1, .axis = Axis::Y })
        .add(TrackingWheel{ .sensor = p2, .axis = Axis::X });

    const auto par  = s.parallel_wheels();
    const auto perp = s.perpendicular_wheels();
    CHECK(par.size()  == 2);
    CHECK(perp.size() == 1);
    CHECK(par[0].sensor.get()  == p1.get());
    CHECK(par[1].sensor.get()  == p2.get());
    CHECK(perp[0].sensor.get() == q1.get());
}

TEST_CASE("Sensors: multiple IMUs supported (spec §6: unlimited IMUs)") {
    auto imu1 = std::make_shared<FakeImu>();
    auto imu2 = std::make_shared<FakeImu>();
    auto imu3 = std::make_shared<FakeImu>();

    auto s = Sensors()
        .add(Imu{ .sensor = imu1, .mounting = ImuMounting::ZUp_XForward })
        .add(Imu{ .sensor = imu2, .mounting = ImuMounting::ZDown_XBackward })
        .add(Imu{ .sensor = imu3, .mounting = ImuMounting::ZUp_XRight });

    REQUIRE(s.imus().size() == 3);
    CHECK(s.imus()[0].mounting == ImuMounting::ZUp_XForward);
    CHECK(s.imus()[1].mounting == ImuMounting::ZDown_XBackward);
    CHECK(s.imus()[2].mounting == ImuMounting::ZUp_XRight);
}

TEST_CASE("Sensors: empty partitions when no tracking wheels added") {
    Sensors s;
    CHECK(s.parallel_wheels().empty());
    CHECK(s.perpendicular_wheels().empty());
}
