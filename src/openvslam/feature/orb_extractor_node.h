#ifndef OPENVSLAM_FEATURE_ORB_EXTRACTOR_NODE_H
#define OPENVSLAM_FEATURE_ORB_EXTRACTOR_NODE_H

#include <list>

#include <opencv2/core/types.hpp>

namespace openvslam {
namespace feature {

class orb_extractor_node {
public:
    orb_extractor_node() = default;

    std::array<orb_extractor_node, 4> divide_node();

    std::vector<cv::KeyPoint> keypts_;

    cv::Point2i pt_begin_, pt_end_;

    std::list<orb_extractor_node>::iterator iter_;

    bool is_leaf_node_ = false;
};

} // namespace feature
} // namespace openvslam

#endif // OPENVSLAM_FEATURE_ORB_EXTRACTOR_NODE_H
