#include "openvslam/feature/orb_extractor.h"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <gtest/gtest.h>

cv::Mat draw_lines(const cv::Mat& img, const unsigned int num_segments = 4) {
    auto lined_img = img.clone();
    for (unsigned int i = 0; i < num_segments; ++i) {
        cv::line(lined_img, cv::Point2i(i * img.cols / num_segments, 0), cv::Point2i(i * img.cols / num_segments, img.rows - 1), cv::Scalar(255, 255, 255), 1);
    }
    for (unsigned int i = 0; i < num_segments; ++i) {
        cv::line(lined_img, cv::Point2i(0, i * img.rows / num_segments), cv::Point2i(img.cols - 1, i * img.rows / num_segments), cv::Scalar(255, 255, 255), 1);
    }
    return lined_img;
}

using namespace openvslam;

TEST(orb_extractor, extract_toy_sample_1) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    auto img = cv::Mat(600, 600, CV_8UC1);
    img = 255;
    cv::rectangle(img, cv::Point2i(300, 300), cv::Point2i(600, 600), cv::Scalar(0), -1, cv::LINE_AA);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // スケールを考慮して誤差を測定
    for (const auto& keypt : keypts) {
        EXPECT_NEAR(keypt.pt.x, 300, 2.0 * extractor.get_scale_factors().at(keypt.octave));
        EXPECT_NEAR(keypt.pt.y, 300, 2.0 * extractor.get_scale_factors().at(keypt.octave));
    }
}

TEST(orb_extractor, extract_toy_sample_2) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    auto img = cv::Mat(2000, 2000, CV_8UC1);
    img = 255;
    cv::rectangle(img, cv::Point2i(0, 0), cv::Point2i(1800, 1800), cv::Scalar(0), -1, cv::LINE_AA);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // スケールを考慮して誤差を測定
    for (const auto& keypt : keypts) {
        EXPECT_NEAR(keypt.pt.x, 1800, 2.0 * extractor.get_scale_factors().at(keypt.octave));
        EXPECT_NEAR(keypt.pt.y, 1800, 2.0 * extractor.get_scale_factors().at(keypt.octave));
    }
}

TEST(orb_extractor, extract_without_mask_1) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg", cv::IMREAD_GRAYSCALE);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);
}

TEST(orb_extractor, extract_without_mask_2) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg", cv::IMREAD_GRAYSCALE);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);
}

TEST(orb_extractor, extract_with_image_mask_1) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg", cv::IMREAD_GRAYSCALE);
    // mask (上下25%をマスク)
    auto mask = cv::Mat(img.rows, img.cols, CV_8UC1);
    mask = 1;
    mask.rowRange(0, img.rows / 4) = 0;
    mask.rowRange(3 * img.rows / 4, img.rows - 1) = 0;

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        EXPECT_GE(keypt.pt.y, img.rows / 4);
        EXPECT_LE(keypt.pt.y, 3 * img.rows / 4);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::imshow("visualization", draw_lines(vis, 8));
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_with_image_mask_2) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg", cv::IMREAD_GRAYSCALE);
    // mask (左右25%をマスク)
    auto mask = cv::Mat(img.rows, img.cols, CV_8UC1);
    mask = 1;
    mask.colRange(0, img.cols / 4) = 0;
    mask.colRange(3 * img.cols / 4, img.cols - 1) = 0;

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        EXPECT_GE(keypt.pt.x, img.cols / 4);
        EXPECT_LE(keypt.pt.x, 3 * img.cols / 4);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::imshow("visualization", draw_lines(vis, 8));
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_with_image_mask_3) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg", cv::IMREAD_GRAYSCALE);
    // mask (画像中央に半径320pixのマスク)
    const unsigned int radius = 320;
    auto mask = cv::Mat(img.rows, img.cols, CV_8UC1);
    mask = 1;
    cv::circle(mask, cv::Point2i(img.cols / 2, img.rows / 2), radius, cv::Scalar(0), -1, cv::LINE_AA);

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        const auto dist_x = keypt.pt.x - img.cols / 2;
        const auto dist_y = keypt.pt.y - img.rows / 2;
        const auto dist_sq = dist_x * dist_x + dist_y * dist_y;
        EXPECT_GE(dist_sq, radius * radius);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::circle(vis, cv::Point2i(img.cols / 2, img.rows / 2), radius, cv::Scalar(255, 255, 255), 1);
        cv::imshow("visualization", vis);
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_with_rectangle_mask_1) {
    auto params = feature::orb_params();
    // mask (上下20%をマスク)
    params.mask_rects_ = {{0.0, 1.0, 0.0, 0.2}, {0.0, 1.0, 0.8, 1.0}};
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg", cv::IMREAD_GRAYSCALE);

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, cv::Mat(), keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        EXPECT_GE(keypt.pt.y, img.rows / 5);
        EXPECT_LE(keypt.pt.y, 4 * img.rows / 5);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::imshow("visualization", draw_lines(vis, 10));
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_with_rectangle_mask_2) {
    auto params = feature::orb_params();
    // mask (上下20%をマスク)
    params.mask_rects_ = {{0.0, 0.2, 0.0, 1.0}, {0.8, 1.0, 0.0, 1.0}};
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg", cv::IMREAD_GRAYSCALE);

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, cv::Mat(), keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        EXPECT_GE(keypt.pt.x, img.cols / 5);
        EXPECT_LE(keypt.pt.x, 4 * img.cols / 5);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::imshow("visualization", draw_lines(vis, 10));
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_with_rectangle_mask_3) {
    auto params = feature::orb_params();
    // mask (上下左右20%をマスク)
    params.mask_rects_ = {{0.0, 0.2, 0.0, 1.0}, {0.8, 1.0, 0.0, 1.0}, {0.0, 1.0, 0.0, 0.2}, {0.0, 1.0, 0.8, 1.0}};
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg", cv::IMREAD_GRAYSCALE);

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, cv::Mat(), keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // mask内に特徴点が存在しないことを確認
    for (const auto& keypt : keypts) {
        EXPECT_GE(keypt.pt.x, img.cols / 5);
        EXPECT_LE(keypt.pt.x, 4 * img.cols / 5);
        EXPECT_GE(keypt.pt.y, img.rows / 5);
        EXPECT_LE(keypt.pt.y, 4 * img.rows / 5);
    }

    const auto show_keypts = std::getenv("SHOW_KEYPOINTS");
    const std::string show_keypts_str = (show_keypts != nullptr) ? show_keypts : "";
    if (show_keypts_str == "YES" || show_keypts_str == "ON" || show_keypts_str == "TRUE" || show_keypts_str == "1") {
        cv::Mat vis;
        cv::drawKeypoints(img, keypts, vis);
        cv::imshow("visualization", draw_lines(vis, 10));
        cv::waitKey(0);
    }
}

TEST(orb_extractor, extract_toy_sample_3) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    auto img = cv::Mat(1200, 600, CV_8UC1);
    img = 255;
    cv::rectangle(img, cv::Point2i(300, 600), cv::Point2i(600, 1200), cv::Scalar(0), -1, cv::LINE_AA);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);

    // スケールを考慮して誤差を測定
    for (const auto& keypt : keypts) {
        EXPECT_NEAR(keypt.pt.x, 300, 2.0 * extractor.get_scale_factors().at(keypt.octave));
        EXPECT_NEAR(keypt.pt.y, 600, 2.0 * extractor.get_scale_factors().at(keypt.octave));
    }
}

TEST(orb_extractor, extract_without_mask_3) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img_land = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_001.jpg", cv::IMREAD_GRAYSCALE);
    // Rotate the landscape image to make it portrait one
    cv::Mat img;
    cv::rotate(img_land, img, cv::ROTATE_90_CLOCKWISE);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);
}

TEST(orb_extractor, extract_without_mask_4) {
    const auto params = feature::orb_params();
    auto extractor = feature::orb_extractor(params);

    // image
    const auto img_land = cv::imread(std::string(TEST_DATA_DIR) + "./equirectangular_image_002.jpg", cv::IMREAD_GRAYSCALE);
    // Rotate the landscape image to make it portrait one
    cv::Mat img;
    cv::rotate(img_land, img, cv::ROTATE_90_CLOCKWISE);
    // mask (無効)
    const auto mask = cv::Mat();

    std::vector<cv::KeyPoint> keypts;
    cv::Mat desc;
    extractor.extract(img, mask, keypts, desc);

    EXPECT_GT(keypts.size(), 0);
    EXPECT_GT(desc.rows, 0);
    EXPECT_EQ(keypts.size(), desc.rows);
    EXPECT_EQ(desc.type(), CV_8U);
}
