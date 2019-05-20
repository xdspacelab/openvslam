#ifndef OPENVSLAM_SOLVER_UTIL_H
#define OPENVSLAM_SOLVER_UTIL_H

#include "openvslam/type.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace solver {

void normalize(const std::vector<cv::KeyPoint>& keypts, std::vector<cv::Point2f>& normalized_pts, Mat33_t& transform);

} // namespace solver
} // namespace openvslam

#endif // OPENVSLAM_SOLVER_UTIL_H
