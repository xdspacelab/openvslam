#include "openvslam/feature/orb_extractor_node.h"

namespace openvslam {
namespace feature {

std::array<orb_extractor_node, 4> orb_extractor_node::divide_node() {
    // 分割後の左上領域の幅と高さ
    const unsigned int half_x = cvCeil((pt_end_.x - pt_begin_.x) / 2.0);
    const unsigned int half_y = cvCeil((pt_end_.y - pt_begin_.y) / 2.0);

    std::array<orb_extractor_node, 4> child_nodes;

    // 分割領域の田の字の中央，上下，左右の座標
    // 左上，右下はself.pt_start_, self.pt_end_
    const auto pt_top = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y);
    const auto pt_left = cv::Point2i(pt_begin_.x, pt_begin_.y + half_y);
    const auto pt_center = cv::Point2i(pt_begin_.x + half_x, pt_begin_.y + half_y);
    const auto pt_right = cv::Point2i(pt_end_.x, pt_begin_.y + half_y);
    const auto pt_bottom = cv::Point2i(pt_begin_.x + half_x, pt_end_.y);

    // 子ノードの領域の左上角と右下角を代入
    child_nodes.at(0).pt_begin_ = pt_begin_;
    child_nodes.at(0).pt_end_ = pt_center;
    child_nodes.at(1).pt_begin_ = pt_top;
    child_nodes.at(1).pt_end_ = pt_right;
    child_nodes.at(2).pt_begin_ = pt_left;
    child_nodes.at(2).pt_end_ = pt_bottom;
    child_nodes.at(3).pt_begin_ = pt_center;
    child_nodes.at(3).pt_end_ = pt_end_;

    // 子ノードのkeypts_のメモリ確保
    for (auto& node: child_nodes) {
        node.keypts_.reserve(keypts_.size());
    }

    // 特徴点を子ノードに分配する
    for (const auto& keypt : keypts_) {
        unsigned int idx = 0;
        if (pt_begin_.x + half_x <= keypt.pt.x) {
            idx += 1;
        }
        if (pt_begin_.y + half_y <= keypt.pt.y) {
            idx += 2;
        }
        child_nodes.at(idx).keypts_.push_back(keypt);
    }

    return child_nodes;
}

} // namespace feature
} // namespace openvslam
