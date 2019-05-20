#include "openvslam/type.h"
#include "openvslam/match/base.h"

#include <chrono>
#include <thread>

#include <gtest/gtest.h>

using namespace openvslam;

TEST(base, compute_hamming_distance_1) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01010101;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b01010101;
    }

    EXPECT_EQ(match::compute_descriptor_distance_32(desc_1, desc_2), 0);
    EXPECT_EQ(match::compute_descriptor_distance_64(desc_1, desc_2), 0);
}

TEST(base, compute_hamming_distance_2) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01010101;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b10101010;
    }

    EXPECT_EQ(match::compute_descriptor_distance_32(desc_1, desc_2), 256);
    EXPECT_EQ(match::compute_descriptor_distance_64(desc_1, desc_2), 256);
}

TEST(base, compute_hamming_distance_3) {
    cv::Mat desc_1(1, 32, CV_8U);
    cv::Mat desc_2(1, 32, CV_8U);

    for (int i = 0; i < desc_1.rows; ++i) {
        desc_1.row(i) = 0b01100110;
    }

    for (int i = 0; i < desc_2.rows; ++i) {
        desc_2.row(i) = 0b00111100;
    }

    EXPECT_EQ(match::compute_descriptor_distance_32(desc_1, desc_2), 128);
    EXPECT_EQ(match::compute_descriptor_distance_64(desc_1, desc_2), 128);
}
