#include "openvslam/util/image_converter.h"

namespace openvslam {
namespace util {

void convert_to_grayscale(cv::Mat& img, const camera::color_order_t in_color_order) {
    if (img.channels() == 3) {
        switch (in_color_order) {
            case camera::color_order_t::Gray: {
                break;
            }
            case camera::color_order_t::RGB: {
                cv::cvtColor(img, img, cv::COLOR_RGB2GRAY);
                break;
            }
            case camera::color_order_t::BGR: {
                cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
                break;
            }
        }
    }
    else if (img.channels() == 4) {
        switch (in_color_order) {
            case camera::color_order_t::Gray: {
                break;
            }
            case camera::color_order_t::RGB: {
                cv::cvtColor(img, img, cv::COLOR_RGBA2GRAY);
                break;
            }
            case camera::color_order_t::BGR: {
                cv::cvtColor(img, img, cv::COLOR_BGRA2GRAY);
                break;
            }
        }
    }
}

void convert_to_true_depth(cv::Mat& img, const double depthmap_factor) {
    img.convertTo(img, CV_32F, 1.0 / depthmap_factor);
}

} // namespace util
} // namespace openvslam
