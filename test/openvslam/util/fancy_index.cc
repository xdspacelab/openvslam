#include "openvslam/type.h"
#include "openvslam/util/fancy_index.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(fancy_index, resample_int) {
    constexpr unsigned int num_elements = 200;
    constexpr unsigned int interval = 4;

    // function for testing
    auto func = [](const unsigned int value) {
        return value * 2 + 1;
    };

    // create original element vector and indices to resample
    std::vector<int> elements(num_elements);
    std::vector<unsigned int> indices;
    std::vector<bool> index_flags(num_elements, false);
    for (unsigned int idx = 0; idx < num_elements; ++idx) {
        elements.at(idx) = func(idx);
        // select the elements whose values can be divided by interval
        if (idx % interval == 0) {
            indices.push_back(idx);
            index_flags.at(idx) = true;
        }
    }

    {
        // resample elements with indices
        const auto resampled = util::resample_by_indices(elements, indices);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }

    {
        // resample elements with flags
        const auto resampled = util::resample_by_indices(elements, index_flags);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }
}

TEST(fancy_index, resample_double) {
    constexpr unsigned int num_elements = 200;
    constexpr unsigned int interval = 1;

    // function for testing
    auto func = [](const double value) {
        return value * 2.5 - 4.6;
    };

    // create original element vector and indices to resample
    std::vector<double> elements(num_elements);
    std::vector<unsigned int> indices;
    std::vector<bool> index_flags(num_elements, false);
    for (unsigned int idx = 0; idx < num_elements; ++idx) {
        elements.at(idx) = func(idx);
        // select the elements whose values can be divided by interval
        if (idx % interval == 0) {
            indices.push_back(idx);
            index_flags.at(idx) = true;
        }
    }

    {
        // resample elements with indices
        const auto resampled = util::resample_by_indices(elements, indices);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }

    {
        // resample elements with flags
        const auto resampled = util::resample_by_indices(elements, index_flags);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }
}

TEST(fancy_index, resample_eigen) {
    constexpr unsigned int num_elements = 200;
    constexpr unsigned int interval = 6;

    // function for testing
    auto func = [](const double value) {
        return Vec3_t{value * 2.5 - 4.6, value * 1.1 + 5.1, value * 4.9 - 0.5};
    };

    // create original element vector and indices to resample
    eigen_alloc_vector<Vec3_t> elements(num_elements);
    std::vector<unsigned int> indices;
    std::vector<bool> index_flags(num_elements, false);
    for (unsigned int idx = 0; idx < num_elements; ++idx) {
        elements.at(idx) = func(idx);
        // select the elements whose values can be divided by interval
        if (idx % interval == 0) {
            indices.push_back(idx);
            index_flags.at(idx) = true;
        }
    }

    {
        // resample elements with indices
        const auto resampled = util::resample_by_indices(elements, indices);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }

    {
        // resample elements with flags
        const auto resampled = util::resample_by_indices(elements, index_flags);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx), elements.at(idx * interval));
        }
    }
}

TEST(fancy_index, resample_keypoint) {
    constexpr unsigned int num_elements = 200;
    constexpr unsigned int interval = 7;

    // function for testing
    auto func = [](const double value) {
        return cv::KeyPoint(value * 1.5, value * 5.1, value / 3.1);
    };

    // create original element vector and indices to resample
    std::vector<cv::KeyPoint> elements(num_elements);
    std::vector<unsigned int> indices;
    std::vector<bool> index_flags(num_elements, false);
    for (unsigned int idx = 0; idx < num_elements; ++idx) {
        elements.at(idx) = func(idx);
        // select the elements whose values can be divided by interval
        if (idx % interval == 0) {
            indices.push_back(idx);
            index_flags.at(idx) = true;
        }
    }

    {
        // resample elements with indices
        const auto resampled = util::resample_by_indices(elements, indices);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx).pt, elements.at(idx * interval).pt);
            EXPECT_EQ(resampled.at(idx).size, elements.at(idx * interval).size);
        }
    }

    {
        // resample elements with indices
        const auto resampled = util::resample_by_indices(elements, index_flags);

        // check vector size
        EXPECT_EQ(resampled.size(), indices.size());
        // check if the specified elements is resampled or not
        for (unsigned int idx = 0; idx < indices.size(); ++idx) {
            EXPECT_EQ(resampled.at(idx).pt, elements.at(idx * interval).pt);
            EXPECT_EQ(resampled.at(idx).size, elements.at(idx * interval).size);
        }
    }
}
