#ifndef OPENVSLAM_MATCH_BASE_H
#define OPENVSLAM_MATCH_BASE_H

#include <array>
#include <algorithm>
#include <numeric>

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace match {

static constexpr unsigned int HAMMING_DIST_THR_LOW = 50;
static constexpr unsigned int HAMMING_DIST_THR_HIGH = 100;
static constexpr unsigned int MAX_HAMMING_DIST = 256;

static constexpr unsigned int HISTOGRAM_LENGTH = 30;
static constexpr float INV_HISTOGRAM_LENGTH = 1.0 / HISTOGRAM_LENGTH;
static constexpr unsigned int NUM_BINS_THR = 3;
static_assert(NUM_BINS_THR <= HISTOGRAM_LENGTH,
              "NUM_BINS_THR should be equal to or smaller than HISTOGRAM_LENGTH");

//! ORB特徴量間のハミング距離を計算する
inline unsigned int compute_descriptor_distance_32(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

    constexpr uint32_t mask_1 = 0x55555555U;
    constexpr uint32_t mask_2 = 0x33333333U;
    constexpr uint32_t mask_3 = 0x0F0F0F0FU;
    constexpr uint32_t mask_4 = 0x01010101U;

    const auto* pa = desc_1.ptr<uint32_t>();
    const auto* pb = desc_2.ptr<uint32_t>();

    unsigned int dist = 0;

    for (unsigned int i = 0; i < 8; ++i, ++pa, ++pb) {
        auto v = *pa ^*pb;
        v -= ((v >> 1) & mask_1);
        v = (v & mask_2) + ((v >> 2) & mask_2);
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 24;
    }

    return dist;
}

//! ORB特徴量間のハミング距離を計算する
inline unsigned int compute_descriptor_distance_64(const cv::Mat& desc_1, const cv::Mat& desc_2) {
    // https://stackoverflow.com/questions/21826292/t-sql-hamming-distance-function-capable-of-decimal-string-uint64?lq=1

    constexpr uint64_t mask_1 = 0x5555555555555555UL;
    constexpr uint64_t mask_2 = 0x3333333333333333UL;
    constexpr uint64_t mask_3 = 0x0F0F0F0F0F0F0F0FUL;
    constexpr uint64_t mask_4 = 0x0101010101010101UL;

    const auto* pa = desc_1.ptr<uint64_t>();
    const auto* pb = desc_2.ptr<uint64_t>();

    unsigned int dist = 0;

    for (unsigned int i = 0; i < 4; ++i, ++pa, ++pb) {
        auto v = *pa ^*pb;
        v -= (v >> 1) & mask_1;
        v = (v & mask_2) + ((v >> 2) & mask_2);
        dist += (((v + (v >> 4)) & mask_3) * mask_4) >> 56;
    }

    return dist;
}

class base {
public:
    base(const float lowe_ratio, const bool check_orientation)
            : lowe_ratio_(lowe_ratio), check_orientation_(check_orientation) {}

    virtual ~base() = default;

protected:
    //! arrayの各要素のvectorのsizeに関して，降順のインデックスソートを行う
    template<typename T, size_t N>
    inline std::array<unsigned int, N> index_sort_by_size(const std::array<std::vector<T>, N> array) const {
        std::array<unsigned int, N> indices;
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
                  [&array](const unsigned int a, const unsigned int b) {
                      return (array.at(a).size() > array.at(b).size());
                  });
        return indices;
    }

    const float lowe_ratio_;
    const bool check_orientation_;
};

} // namespace match
} // namespace openvslam

#endif // OPENVSLAM_MATCH_BASE_H
