#ifndef OPENVSLAM_UTIL_IMAGE_CONVERTER_H
#define OPENVSLAM_UTIL_IMAGE_CONVERTER_H

#include "openvslam/camera/base.h"

#include <opencv2/core.hpp>

namespace openvslam {
namespace util {

void convert_to_grayscale(cv::Mat& img, const camera::color_order_t in_color_order);

void convert_to_true_depth(cv::Mat& img, const double depthmap_factor);

void equalize_histogram(cv::Mat& img);

} // namespace util
} // namespace openvslam

#endif // OPENVSLAM_UTIL_IMAGE_CONVERTER_H
