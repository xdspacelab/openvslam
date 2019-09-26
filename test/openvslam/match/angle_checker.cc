#include "openvslam/match/angle_checker.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(angle_checker, check_assert) {
    EXPECT_NO_THROW(match::angle_checker<int>(30, 3));
    EXPECT_NO_THROW(match::angle_checker<int>(30, 30));
}

TEST(angle_checker, check_matches_top_1) {
    match::angle_checker<int> angle_checker(30, 1);

    // set five matches around 35.0 deg
    angle_checker.append_delta_angle(34.8, 35);
    angle_checker.append_delta_angle(34.9, 35);
    angle_checker.append_delta_angle(35.0, 35);
    angle_checker.append_delta_angle(35.1, 35);
    angle_checker.append_delta_angle(35.2, 35);

    // set four matches around 324.0 deg
    angle_checker.append_delta_angle(323.8, 324);
    angle_checker.append_delta_angle(323.9, 324);
    angle_checker.append_delta_angle(324.1, 324);
    angle_checker.append_delta_angle(324.2, 324);

    // set three matches around 127.0 deg
    angle_checker.append_delta_angle(126.9, 127);
    angle_checker.append_delta_angle(127.0, 127);
    angle_checker.append_delta_angle(127.1, 127);

    // scatter other matches
    angle_checker.append_delta_angle(0.0, 0);
    angle_checker.append_delta_angle(90.0, 60);
    angle_checker.append_delta_angle(180.0, 180);
    angle_checker.append_delta_angle(270.0, 270);

    // check valid_matches contains 35
    const auto valid_matches = angle_checker.get_valid_matches();
    EXPECT_TRUE(!valid_matches.empty());
    for (const auto match : valid_matches) {
        EXPECT_TRUE(match == 35);
        EXPECT_TRUE(match != 324 && match != 127 && match != 0 && match != 60 && match != 180 && match != 270);
    }

    // check invalid_matches does not contain 35
    const auto invalid_matches = angle_checker.get_invalid_matches();
    for (const auto match : invalid_matches) {
        EXPECT_TRUE(match != 35);
        EXPECT_TRUE(match == 324 || match == 127 || match == 0 || match == 60 || match == 180 || match == 270);
    }
}

TEST(angle_checker, check_matches_top_2) {
    match::angle_checker<int> angle_checker(30, 2);

    // set five matches around 35.0 deg
    angle_checker.append_delta_angle(34.8, 35);
    angle_checker.append_delta_angle(34.9, 35);
    angle_checker.append_delta_angle(35.0, 35);
    angle_checker.append_delta_angle(35.1, 35);
    angle_checker.append_delta_angle(35.2, 35);

    // set four matches around 324.0 deg
    angle_checker.append_delta_angle(323.8, 324);
    angle_checker.append_delta_angle(323.9, 324);
    angle_checker.append_delta_angle(324.1, 324);
    angle_checker.append_delta_angle(324.2, 324);

    // set three matches around 127.0 deg
    angle_checker.append_delta_angle(126.9, 127);
    angle_checker.append_delta_angle(127.0, 127);
    angle_checker.append_delta_angle(127.1, 127);

    // scatter other matches
    angle_checker.append_delta_angle(0.0, 0);
    angle_checker.append_delta_angle(90.0, 60);
    angle_checker.append_delta_angle(180.0, 180);
    angle_checker.append_delta_angle(270.0, 270);

    // check valid_matches contains 35 and 324
    const auto valid_matches = angle_checker.get_valid_matches();
    EXPECT_TRUE(!valid_matches.empty());
    for (const auto match : valid_matches) {
        EXPECT_TRUE(match == 35 || match == 324);
        EXPECT_TRUE(match != 127 && match != 0 && match != 60 && match != 180 && match != 270);
    }

    // check invalid_matches does not contain 35 and 324
    const auto invalid_matches = angle_checker.get_invalid_matches();
    EXPECT_TRUE(!invalid_matches.empty());
    for (const auto match : invalid_matches) {
        EXPECT_TRUE(match != 35 && match != 324);
        EXPECT_TRUE(match == 127 || match == 0 || match == 60 || match == 180 || match == 270);
    }
}

TEST(angle_checker, check_matches_top_3) {
    match::angle_checker<int> angle_checker(30, 3);

    // set five matches around 35.0 deg
    angle_checker.append_delta_angle(34.8, 35);
    angle_checker.append_delta_angle(34.9, 35);
    angle_checker.append_delta_angle(35.0, 35);
    angle_checker.append_delta_angle(35.1, 35);
    angle_checker.append_delta_angle(35.2, 35);

    // set four matches around 324.0 deg
    angle_checker.append_delta_angle(323.8, 324);
    angle_checker.append_delta_angle(323.9, 324);
    angle_checker.append_delta_angle(324.1, 324);
    angle_checker.append_delta_angle(324.2, 324);

    // set three matches around 127.0 deg
    angle_checker.append_delta_angle(126.9, 127);
    angle_checker.append_delta_angle(127.0, 127);
    angle_checker.append_delta_angle(127.1, 127);

    // scatter other matches
    angle_checker.append_delta_angle(0.0, 0);
    angle_checker.append_delta_angle(90.0, 60);
    angle_checker.append_delta_angle(180.0, 180);
    angle_checker.append_delta_angle(270.0, 270);

    // check valid_matches contains 35, 324, and 127
    const auto valid_matches = angle_checker.get_valid_matches();
    EXPECT_TRUE(!valid_matches.empty());
    for (const auto match : valid_matches) {
        EXPECT_TRUE(match == 35 || match == 324 || match == 127);
        EXPECT_TRUE(match != 0 && match != 60 && match != 180 && match != 270);
    }

    // check invalid_matches does not contain 35, 324, and 127
    const auto invalid_matches = angle_checker.get_invalid_matches();
    EXPECT_TRUE(!invalid_matches.empty());
    for (const auto match : invalid_matches) {
        EXPECT_TRUE(match != 35 && match != 324 && match != 127);
        EXPECT_TRUE(match == 0 || match == 60 || match == 180 || match == 270);
    }
}

TEST(angle_checker, check_matches_all) {
    match::angle_checker<int> angle_checker(30, 30);

    // set five matches around 35.0 deg
    angle_checker.append_delta_angle(34.8, 35);
    angle_checker.append_delta_angle(34.9, 35);
    angle_checker.append_delta_angle(35.0, 35);
    angle_checker.append_delta_angle(35.1, 35);
    angle_checker.append_delta_angle(35.2, 35);

    // set four matches around 324.0 deg
    angle_checker.append_delta_angle(323.8, 324);
    angle_checker.append_delta_angle(323.9, 324);
    angle_checker.append_delta_angle(324.1, 324);
    angle_checker.append_delta_angle(324.2, 324);

    // set three matches around 127.0 deg
    angle_checker.append_delta_angle(126.9, 127);
    angle_checker.append_delta_angle(127.0, 127);
    angle_checker.append_delta_angle(127.1, 127);

    // scatter other matches
    angle_checker.append_delta_angle(0.0, 0);
    angle_checker.append_delta_angle(90.0, 60);
    angle_checker.append_delta_angle(180.0, 180);
    angle_checker.append_delta_angle(270.0, 270);

    // check valid_matches contains all of the entries
    const auto valid_matches = angle_checker.get_valid_matches();
    EXPECT_TRUE(!valid_matches.empty());
    for (const auto match : valid_matches) {
        EXPECT_TRUE(match == 35 || match == 324 || match == 127 || match == 0 || match == 60 || match == 180 || match == 270);
    }

    // check invalid_matches contains no entries
    const auto invalid_matches = angle_checker.get_invalid_matches();
    EXPECT_TRUE(invalid_matches.empty());
}
