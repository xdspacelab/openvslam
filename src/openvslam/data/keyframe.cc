#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/camera/radial_division.h"
#include "openvslam/data/common.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/feature/orb_params.h"
#include "openvslam/util/converter.h"

#include <nlohmann/json.hpp>

namespace openvslam {
namespace data {

std::atomic<unsigned int> keyframe::next_id_{0};

keyframe::keyframe(const frame& frm, map_database* map_db, bow_database* bow_db)
    : // meta information
      id_(next_id_++), src_frm_id_(frm.id_), timestamp_(frm.timestamp_),
      // camera parameters
      camera_(frm.camera_), depth_thr_(frm.depth_thr_),
      // constant observations
      num_keypts_(frm.num_keypts_), keypts_(frm.keypts_), undist_keypts_(frm.undist_keypts_), bearings_(frm.bearings_),
      keypt_indices_in_cells_(frm.keypt_indices_in_cells_),
      stereo_x_right_(frm.stereo_x_right_), depths_(frm.depths_), descriptors_(frm.descriptors_.clone()),
      // BoW
      bow_vec_(frm.bow_vec_), bow_feat_vec_(frm.bow_feat_vec_),
      // covisibility graph node (connections is not assigned yet)
      graph_node_(std::unique_ptr<graph_node>(new graph_node(this, true))),
      // ORB scale pyramid
      num_scale_levels_(frm.num_scale_levels_), scale_factor_(frm.scale_factor_),
      log_scale_factor_(frm.log_scale_factor_), scale_factors_(frm.scale_factors_),
      level_sigma_sq_(frm.level_sigma_sq_), inv_level_sigma_sq_(frm.inv_level_sigma_sq_),
      // observations
      landmarks_(frm.landmarks_),
      // databases
      map_db_(map_db), bow_db_(bow_db), bow_vocab_(frm.bow_vocab_) {
    // set pose parameters (cam_pose_wc_, cam_center_) using frm.cam_pose_cw_
    set_cam_pose(frm.cam_pose_cw_);
}

keyframe::keyframe(const unsigned int id, const unsigned int src_frm_id, const double timestamp,
                   const Mat44_t& cam_pose_cw, camera::base* camera, const float depth_thr,
                   const unsigned int num_keypts, const std::vector<cv::KeyPoint>& keypts,
                   const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
                   const std::vector<float>& stereo_x_right, const std::vector<float>& depths, const cv::Mat& descriptors,
                   const unsigned int num_scale_levels, const float scale_factor,
                   bow_vocabulary* bow_vocab, bow_database* bow_db, map_database* map_db)
    : // meta information
      id_(id), src_frm_id_(src_frm_id), timestamp_(timestamp),
      // camera parameters
      camera_(camera), depth_thr_(depth_thr),
      // constant observations
      num_keypts_(num_keypts), keypts_(keypts), undist_keypts_(undist_keypts), bearings_(bearings),
      keypt_indices_in_cells_(assign_keypoints_to_grid(camera, undist_keypts)),
      stereo_x_right_(stereo_x_right), depths_(depths), descriptors_(descriptors.clone()),
      // graph node (connections is not assigned yet)
      graph_node_(std::unique_ptr<graph_node>(new graph_node(this, false))),
      // ORB scale pyramid
      num_scale_levels_(num_scale_levels), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)),
      scale_factors_(feature::orb_params::calc_scale_factors(num_scale_levels, scale_factor)),
      level_sigma_sq_(feature::orb_params::calc_level_sigma_sq(num_scale_levels, scale_factor)),
      inv_level_sigma_sq_(feature::orb_params::calc_inv_level_sigma_sq(num_scale_levels, scale_factor)),
      // others
      landmarks_(std::vector<landmark*>(num_keypts, nullptr)),
      // databases
      map_db_(map_db), bow_db_(bow_db), bow_vocab_(bow_vocab) {
    // compute BoW (bow_vec_, bow_feat_vec_) using descriptors_
    compute_bow();
    // set pose parameters (cam_pose_wc_, cam_center_) using cam_pose_cw_
    set_cam_pose(cam_pose_cw);

    // TODO: should set the pointers of landmarks_ using add_landmark()

    // TODO: should compute connected_keyfrms_and_weights_
    // TODO: should compute ordered_connected_keyfrms_
    // TODO: should compute ordered_weights_

    // TODO: should set spanning_parent_ using set_spanning_parent()
    // TODO: should set spanning_children_ using add_spanning_child()
    // TODO: should set loop_edges_ using add_loop_edge()
}

nlohmann::json keyframe::to_json() const {
    // extract landmark IDs
    std::vector<int> landmark_ids(landmarks_.size(), -1);
    for (unsigned int i = 0; i < landmark_ids.size(); ++i) {
        if (landmarks_.at(i) && !landmarks_.at(i)->will_be_erased()) {
            landmark_ids.at(i) = landmarks_.at(i)->id_;
        }
    }

    // extract spanning tree parent
    auto spanning_parent = graph_node_->get_spanning_parent();

    // extract spanning tree children
    const auto spanning_children = graph_node_->get_spanning_children();
    std::vector<int> spanning_child_ids;
    spanning_child_ids.reserve(spanning_children.size());
    for (const auto spanning_child : spanning_children) {
        spanning_child_ids.push_back(spanning_child->id_);
    }

    // extract loop edges
    const auto loop_edges = graph_node_->get_loop_edges();
    std::vector<int> loop_edge_ids;
    for (const auto loop_edge : loop_edges) {
        loop_edge_ids.push_back(loop_edge->id_);
    }

    return {{"src_frm_id", src_frm_id_},
            {"ts", timestamp_},
            {"cam", camera_->name_},
            {"depth_thr", depth_thr_},
            // camera pose
            {"rot_cw", convert_rotation_to_json(cam_pose_cw_.block<3, 3>(0, 0))},
            {"trans_cw", convert_translation_to_json(cam_pose_cw_.block<3, 1>(0, 3))},
            // features and observations
            {"n_keypts", num_keypts_},
            {"keypts", convert_keypoints_to_json(keypts_)},
            {"undists", convert_undistorted_to_json(undist_keypts_)},
            {"x_rights", stereo_x_right_},
            {"depths", depths_},
            {"descs", convert_descriptors_to_json(descriptors_)},
            {"lm_ids", landmark_ids},
            // orb scale information
            {"n_scale_levels", num_scale_levels_},
            {"scale_factor", scale_factor_},
            // graph information
            {"span_parent", spanning_parent ? spanning_parent->id_ : -1},
            {"span_children", spanning_child_ids},
            {"loop_edges", loop_edge_ids}};
}

void keyframe::set_cam_pose(const Mat44_t& cam_pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    cam_pose_cw_ = cam_pose_cw;

    const Mat33_t rot_cw = cam_pose_cw_.block<3, 3>(0, 0);
    const Vec3_t trans_cw = cam_pose_cw_.block<3, 1>(0, 3);
    const Mat33_t rot_wc = rot_cw.transpose();
    cam_center_ = -rot_wc * trans_cw;

    cam_pose_wc_ = Mat44_t::Identity();
    cam_pose_wc_.block<3, 3>(0, 0) = rot_wc;
    cam_pose_wc_.block<3, 1>(0, 3) = cam_center_;
}

void keyframe::set_cam_pose(const g2o::SE3Quat& cam_pose_cw) {
    set_cam_pose(util::converter::to_eigen_mat(cam_pose_cw));
}

Mat44_t keyframe::get_cam_pose() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return cam_pose_cw_;
}

Mat44_t keyframe::get_cam_pose_inv() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return cam_pose_wc_;
}

Vec3_t keyframe::get_cam_center() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return cam_center_;
}

Mat33_t keyframe::get_rotation() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return cam_pose_cw_.block<3, 3>(0, 0);
}

Vec3_t keyframe::get_translation() const {
    std::lock_guard<std::mutex> lock(mtx_pose_);
    return cam_pose_cw_.block<3, 1>(0, 3);
}

void keyframe::compute_bow() {
    if (bow_vec_.empty() || bow_feat_vec_.empty()) {
#ifdef USE_DBOW2
        bow_vocab_->transform(util::converter::to_desc_vec(descriptors_), bow_vec_, bow_feat_vec_, 4);
#else
        bow_vocab_->transform(descriptors_, 4, bow_vec_, bow_feat_vec_);
#endif
    }
}

void keyframe::add_landmark(landmark* lm, const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.at(idx) = lm;
}

void keyframe::erase_landmark_with_index(const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.at(idx) = nullptr;
}

void keyframe::erase_landmark(landmark* lm) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    int idx = lm->get_index_in_keyframe(this);
    if (0 <= idx) {
        landmarks_.at(static_cast<unsigned int>(idx)) = nullptr;
    }
}

void keyframe::replace_landmark(landmark* lm, const unsigned int idx) {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    landmarks_.at(idx) = lm;
}

std::vector<landmark*> keyframe::get_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return landmarks_;
}

std::set<landmark*> keyframe::get_valid_landmarks() const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    std::set<landmark*> valid_landmarks;

    for (const auto lm : landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        valid_landmarks.insert(lm);
    }

    return valid_landmarks;
}

unsigned int keyframe::get_num_tracked_landmarks(const unsigned int min_num_obs_thr) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    unsigned int num_tracked_lms = 0;

    if (0 < min_num_obs_thr) {
        for (const auto lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (min_num_obs_thr <= lm->num_observations()) {
                ++num_tracked_lms;
            }
        }
    }
    else {
        for (const auto lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            ++num_tracked_lms;
        }
    }

    return num_tracked_lms;
}

landmark* keyframe::get_landmark(const unsigned int idx) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    return landmarks_.at(idx);
}

std::vector<unsigned int> keyframe::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin) const {
    return data::get_keypoints_in_cell(camera_, undist_keypts_, keypt_indices_in_cells_, ref_x, ref_y, margin);
}

Vec3_t keyframe::triangulate_stereo(const unsigned int idx) const {
    assert(camera_->setup_type_ != camera::setup_type_t::Monocular);

    switch (camera_->model_type_) {
        case camera::model_type_t::Perspective: {
            auto camera = static_cast<camera::perspective*>(camera_);

            const float depth = depths_.at(idx);
            if (0.0 < depth) {
                const float x = undist_keypts_.at(idx).pt.x;
                const float y = undist_keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                std::lock_guard<std::mutex> lock(mtx_pose_);
                return cam_pose_wc_.block<3, 3>(0, 0) * pos_c + cam_pose_wc_.block<3, 1>(0, 3);
            }
            else {
                return Vec3_t::Zero();
            }
        }
        case camera::model_type_t::Fisheye: {
            auto camera = static_cast<camera::fisheye*>(camera_);

            const float depth = depths_.at(idx);
            if (0.0 < depth) {
                const float x = undist_keypts_.at(idx).pt.x;
                const float y = undist_keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                std::lock_guard<std::mutex> lock(mtx_pose_);
                return cam_pose_wc_.block<3, 3>(0, 0) * pos_c + cam_pose_wc_.block<3, 1>(0, 3);
            }
            else {
                return Vec3_t::Zero();
            }
        }
        case camera::model_type_t::Equirectangular: {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
        }
        case camera::model_type_t::RadialDivision: {
            auto camera = static_cast<camera::radial_division*>(camera_);

            const float depth = depths_.at(idx);
            if (0.0 < depth) {
                const float x = keypts_.at(idx).pt.x;
                const float y = keypts_.at(idx).pt.y;
                const float unproj_x = (x - camera->cx_) * depth * camera->fx_inv_;
                const float unproj_y = (y - camera->cy_) * depth * camera->fy_inv_;
                const Vec3_t pos_c{unproj_x, unproj_y, depth};

                std::lock_guard<std::mutex> lock(mtx_pose_);
                // camera座標 -> world座標
                return cam_pose_wc_.block<3, 3>(0, 0) * pos_c + cam_pose_wc_.block<3, 1>(0, 3);
            }
            else {
                return Vec3_t::Zero();
            }
        }
    }

    return Vec3_t::Zero();
}

float keyframe::compute_median_depth(const bool abs) const {
    std::vector<landmark*> landmarks;
    Mat44_t cam_pose_cw;
    {
        std::lock_guard<std::mutex> lock1(mtx_observations_);
        std::lock_guard<std::mutex> lock2(mtx_pose_);
        landmarks = landmarks_;
        cam_pose_cw = cam_pose_cw_;
    }

    std::vector<float> depths;
    depths.reserve(num_keypts_);
    const Vec3_t rot_cw_z_row = cam_pose_cw.block<1, 3>(2, 0);
    const float trans_cw_z = cam_pose_cw(2, 3);

    for (const auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        const Vec3_t pos_w = lm->get_pos_in_world();
        const auto pos_c_z = rot_cw_z_row.dot(pos_w) + trans_cw_z;
        depths.push_back(abs ? std::abs(pos_c_z) : pos_c_z);
    }

    std::sort(depths.begin(), depths.end());

    return depths.at((depths.size() - 1) / 2);
}

void keyframe::set_not_to_be_erased() {
    cannot_be_erased_ = true;
}

void keyframe::set_to_be_erased() {
    if (!graph_node_->has_loop_edge()) {
        cannot_be_erased_ = false;
    }
}

void keyframe::prepare_for_erasing() {
    // cannot erase the origin
    if (*this == *(map_db_->origin_keyfrm_)) {
        return;
    }

    // cannot erase if the frag is raised
    if (cannot_be_erased_) {
        return;
    }

    // 1. raise the flag which indicates it has been erased

    will_be_erased_ = true;

    // 2. remove associations between keypoints and landmarks

    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        for (const auto lm : landmarks_) {
            if (!lm) {
                continue;
            }
            lm->erase_observation(this);
        }
    }

    // 3. recover covisibility graph and spanning tree

    // remove covisibility information
    graph_node_->erase_all_connections();
    // recover spanning tree
    graph_node_->recover_spanning_connections();

    // 3. update frame statistics

    map_db_->replace_reference_keyframe(this, graph_node_->get_spanning_parent());

    // 4. remove myself from the databased

    map_db_->erase_keyframe(this);
    bow_db_->erase_keyframe(this);
}

bool keyframe::will_be_erased() {
    return will_be_erased_;
}

} // namespace data
} // namespace openvslam
