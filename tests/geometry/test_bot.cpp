#include <doctest/doctest.h>

#include <pathfinder/geometry/bot.hpp>
#include <pathfinder/geometry/footprint.hpp>
#include <pathfinder/geometry/params.hpp>

using namespace pathfinder;

TEST_CASE("Bot: default origin is BackLeft") {
    Bot b;
    CHECK(b.origin() == Corner::BackLeft);
}

TEST_CASE("Bot: footprint setter and getter") {
    Bot b;
    b.footprint(18.0, 12.0);
    CHECK(b.footprint().length == doctest::Approx(18.0));
    CHECK(b.footprint().width == doctest::Approx(12.0));
}

TEST_CASE("Bot: builder fluency returns self") {
    Bot b;
    Bot& ref = b.footprint(18.0, 18.0).origin(Corner::FrontRight);
    CHECK(&ref == &b);
    CHECK(b.origin() == Corner::FrontRight);
}

TEST_CASE("Bot: chained builder construction (spec example)") {
    auto bot = Bot()
        .footprint(18.0, 18.0)
        .origin(Corner::BackLeft)
        .point("intake", forward = 5.0, right = 10.0)
        .point("claw",   forward = 15.0, left = 9.0)
        .point("camera", forward = 8.0, right = 0.0);

    CHECK(bot.footprint().length == doctest::Approx(18.0));
    CHECK(bot.origin() == Corner::BackLeft);

    const auto& intake = bot.point("intake");
    CHECK(intake.name == "intake");
    CHECK(intake.offset == Vector2{5.0, 10.0});

    const auto& claw = bot.point("claw");
    CHECK(claw.offset == Vector2{15.0, -9.0});

    const auto& camera = bot.point("camera");
    CHECK(camera.offset == Vector2{8.0, 0.0});
}

TEST_CASE("Bot: lookup unknown point throws") {
    auto bot = Bot().footprint(18, 18);
    CHECK_THROWS_AS(bot.point("nonexistent"), std::out_of_range);
}

TEST_CASE("Bot: multiple points with same prefix") {
    auto bot = Bot()
        .footprint(18, 18)
        .point("intake_left", forward = 5.0, left = 6.0)
        .point("intake_right", forward = 5.0, right = 6.0);

    CHECK(bot.point("intake_left").offset == Vector2{5.0, -6.0});
    CHECK(bot.point("intake_right").offset == Vector2{5.0, 6.0});
}

TEST_CASE("Bot: raw x_/y_ point") {
    auto bot = Bot().footprint(18, 18).point("raw", x_ = 3.0, y_ = -4.0);
    CHECK(bot.point("raw").offset == Vector2{3.0, -4.0});
}

TEST_CASE("Bot: points() exposes underlying vector") {
    auto bot = Bot()
        .footprint(18, 18)
        .point("a", forward = 1.0, right = 2.0)
        .point("b", forward = 3.0, right = 4.0);
    CHECK(bot.points().size() == 2);
    CHECK(bot.points()[0].name == "a");
    CHECK(bot.points()[1].name == "b");
}
