#include <doctest/doctest.h>

#include <pathfinder/geometry/angle.hpp>
#include <pathfinder/sensors/imu.hpp>
#include <pathfinder/sensors/mocks.hpp>

#include <memory>

using namespace pathfinder;

namespace {

Imu make_imu(ImuMounting mounting, std::shared_ptr<FakeImu> fake) {
    Imu imu;
    imu.sensor   = fake;
    imu.mounting = mounting;
    return imu;
}

} // namespace

TEST_CASE("Imu: ZUp_XForward flips heading sign (RH/CCW → CW)") {
    auto fake = std::make_shared<FakeImu>();
    auto imu  = make_imu(ImuMounting::ZUp_XForward, fake);

    fake->set_heading(Angle::degrees(30.0));     // raw IMU 30° CCW
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(-30.0));

    fake->set_heading(Angle::degrees(-90.0));
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(90.0));

    fake->set_heading(Angle{});
    CHECK(imu.heading_in_bot_frame().rad == doctest::Approx(0.0));
}

TEST_CASE("Imu: ZUp_XForward flips yaw-rate sign") {
    auto fake = std::make_shared<FakeImu>();
    auto imu  = make_imu(ImuMounting::ZUp_XForward, fake);

    fake->set_yaw_rate(1.0);
    CHECK(imu.yaw_rate_in_bot_frame() == doctest::Approx(-1.0));

    fake->set_yaw_rate(-2.5);
    CHECK(imu.yaw_rate_in_bot_frame() == doctest::Approx(2.5));
}

TEST_CASE("Imu: all ZUp_* mountings flip yaw sign") {
    auto fake = std::make_shared<FakeImu>();
    fake->set_yaw_rate(0.7);

    for (auto m : {ImuMounting::ZUp_XForward,
                   ImuMounting::ZUp_XBackward,
                   ImuMounting::ZUp_XLeft,
                   ImuMounting::ZUp_XRight}) {
        auto imu = make_imu(m, fake);
        CHECK(imu.yaw_rate_in_bot_frame() == doctest::Approx(-0.7));
    }
}

TEST_CASE("Imu: ZDown_XForward preserves heading sign") {
    auto fake = std::make_shared<FakeImu>();
    auto imu  = make_imu(ImuMounting::ZDown_XForward, fake);

    fake->set_heading(Angle::degrees(45.0));
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(45.0));

    fake->set_heading(Angle::degrees(-120.0));
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(-120.0));
}

TEST_CASE("Imu: ZDown_XForward preserves yaw-rate sign") {
    auto fake = std::make_shared<FakeImu>();
    auto imu  = make_imu(ImuMounting::ZDown_XForward, fake);

    fake->set_yaw_rate(1.5);
    CHECK(imu.yaw_rate_in_bot_frame() == doctest::Approx(1.5));
}

TEST_CASE("Imu: all ZDown_* mountings preserve yaw sign") {
    auto fake = std::make_shared<FakeImu>();
    fake->set_yaw_rate(-0.3);

    for (auto m : {ImuMounting::ZDown_XForward,
                   ImuMounting::ZDown_XBackward,
                   ImuMounting::ZDown_XLeft,
                   ImuMounting::ZDown_XRight}) {
        auto imu = make_imu(m, fake);
        CHECK(imu.yaw_rate_in_bot_frame() == doctest::Approx(-0.3));
    }
}

TEST_CASE("Imu: heading is normalized to (-π, π]") {
    auto fake = std::make_shared<FakeImu>();
    auto imu  = make_imu(ImuMounting::ZDown_XForward, fake);

    // 200° in raw → bot heading 200° → normalized to -160°.
    fake->set_heading(Angle::degrees(200.0));
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(-160.0));

    // -200° in raw → bot heading -200° → normalized to +160°.
    fake->set_heading(Angle::degrees(-200.0));
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(160.0));
}

TEST_CASE("Imu: ZUp_XBackward is the same yaw sign as ZUp_XForward") {
    // The X-direction part of the mounting changes the heading reference; the
    // user is expected to zero the IMU at the bot's reference heading, which
    // absorbs that constant. The *sign* multiplier is unaffected by X-orient.
    auto fake = std::make_shared<FakeImu>();
    fake->set_heading(Angle::degrees(50.0));

    auto imu_fwd  = make_imu(ImuMounting::ZUp_XForward,  fake);
    auto imu_back = make_imu(ImuMounting::ZUp_XBackward, fake);
    CHECK(imu_fwd.heading_in_bot_frame().degrees()
        == doctest::Approx(imu_back.heading_in_bot_frame().degrees()));
}

TEST_CASE("Imu: FakeImu calibration and reset behaviors") {
    auto fake = std::make_shared<FakeImu>();
    CHECK(fake->is_calibrating() == false);
    fake->set_calibrating(true);
    CHECK(fake->is_calibrating() == true);

    fake->set_heading(Angle::degrees(75.0));
    fake->reset_heading();
    CHECK(fake->heading_rad().rad == doctest::Approx(0.0));

    fake->reset_heading(Angle::degrees(20.0));
    CHECK(fake->heading_rad().degrees() == doctest::Approx(20.0));
}

TEST_CASE("Imu: full path uses sensor pointer through Imu wrapper") {
    auto fake = std::make_shared<FakeImu>();
    Imu imu{
        .sensor   = fake,
        .mounting = ImuMounting::ZDown_XForward,
    };

    fake->set_heading(Angle::degrees(15.0));
    fake->set_yaw_rate(0.4);
    CHECK(imu.heading_in_bot_frame().degrees() == doctest::Approx(15.0));
    CHECK(imu.yaw_rate_in_bot_frame()          == doctest::Approx(0.4));
}
