#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/data/common.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/feature/orb_params.h"
#include "openvslam/util/converter.h"

namespace openvslam {
namespace data {

std::atomic<unsigned int> keyframe::next_id_{0};

keyframe::keyframe(const frame& frm, map_database* map_db, bow_database* bow_db) :
        id_(next_id_++), src_frm_id_(frm.id_), timestamp_(frm.timestamp_),
        // camera
        camera_(frm.camera_), depth_thr_(frm.depth_thr_),
        // features
        num_keypts_(frm.num_keypts_), keypts_(frm.keypts_), undist_keypts_(frm.undist_keypts_), bearings_(frm.bearings_),
        stereo_x_right_(frm.stereo_x_right_), depths_(frm.depths_), descriptors_(frm.descriptors_.clone()),
        // BoW
        bow_vec_(frm.bow_vec_), bow_feat_vec_(frm.bow_feat_vec_),
        // ORB scale pyramid
        num_scale_levels_(frm.num_scale_levels_), scale_factor_(frm.scale_factor_),
        log_scale_factor_(frm.log_scale_factor_), scale_factors_(frm.scale_factors_),
        level_sigma_sq_(frm.level_sigma_sq_), inv_level_sigma_sq_(frm.inv_level_sigma_sq_),
        // others
        landmarks_(frm.landmarks_), bow_vocab_(frm.bow_vocab_), bow_db_(bow_db),
        keypt_indices_in_cells_(frm.keypt_indices_in_cells_), map_db_(map_db) {
    set_cam_pose(frm.cam_pose_cw_);
}

keyframe::keyframe(const unsigned int id, const unsigned int src_frm_id, const double timestamp,
                   const Mat44_t& cam_pose_cw, camera::base* camera, const float depth_thr,
                   const unsigned int num_keypts, const std::vector<cv::KeyPoint>& keypts,
                   const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
                   const std::vector<float>& stereo_x_right, const std::vector<float>& depths, const cv::Mat& descriptors,
                   const unsigned int num_scale_levels, const float scale_factor,
                   bow_vocabulary* bow_vocab, bow_database* bow_db, map_database* map_db) :
        id_(id), src_frm_id_(src_frm_id), timestamp_(timestamp),
        // camera
        camera_(camera), depth_thr_(depth_thr),
        // features
        num_keypts_(num_keypts), keypts_(keypts), undist_keypts_(undist_keypts), bearings_(bearings),
        stereo_x_right_(stereo_x_right), depths_(depths), descriptors_(descriptors.clone()),
        // ORB scale pyramid
        num_scale_levels_(num_scale_levels), scale_factor_(scale_factor), log_scale_factor_(std::log(scale_factor)),
        scale_factors_(feature::orb_params::calc_scale_factors(num_scale_levels, scale_factor)),
        level_sigma_sq_(feature::orb_params::calc_level_sigma_sq(num_scale_levels, scale_factor)),
        inv_level_sigma_sq_(feature::orb_params::calc_inv_level_sigma_sq(num_scale_levels, scale_factor)),
        // others
        landmarks_(std::vector<landmark*>(num_keypts, nullptr)), bow_vocab_(bow_vocab), bow_db_(bow_db),
        is_first_connection_(false), map_db_(map_db) {
    // compute BoW (bow_vec_, bow_feat_vec_) using descriptors_
    compute_bow();
    // set pose parameters (cam_pose_wc_, cam_center_) using cam_pose_cw_
    set_cam_pose(cam_pose_cw);
    // compute cell indices of keypoints
    assign_keypoints_to_grid(camera_, undist_keypts_, keypt_indices_in_cells_);

    // TODO: should set the pointers of landmarks_ using add_landmark()

    // TODO: should compute connected_keyfrms_and_weights_
    // TODO: should compute ordered_connected_keyfrms_
    // TODO: should compute ordered_weights_

    // TODO: should set spanning_parent_ using set_spanning_parent()
    // TODO: should set spanning_children_ using add_spanning_child()
    // TODO: should set loop_edges_ using add_loop_edge()
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

void keyframe::add_connection(keyframe* keyfrm, const unsigned int weight) {
    bool need_update = false;
    {
        std::lock_guard<std::mutex> lock(mtx_connections_);
        if (!connected_keyfrms_and_weights_.count(keyfrm)) {
            // なければ追加
            connected_keyfrms_and_weights_[keyfrm] = weight;
            need_update = true;
        }
        else if (connected_keyfrms_and_weights_.at(keyfrm) != weight) {
            // weightが変わっていれば更新
            connected_keyfrms_and_weights_.at(keyfrm) = weight;
            need_update = true;
        }
    }

    // 追加・更新が行われた場合のみグラフの更新が必要
    if (need_update) {
        update_covisibility_orders();
    }
}

void keyframe::erase_connection(keyframe* keyfrm) {
    bool need_update = false;
    {
        std::lock_guard<std::mutex> lock(mtx_connections_);
        if (connected_keyfrms_and_weights_.count(keyfrm)) {
            connected_keyfrms_and_weights_.erase(keyfrm);
            need_update = true;
        }
    }

    // 削除が行われた場合のみグラフの更新が必要
    if (need_update) {
        update_covisibility_orders();
    }
}

void keyframe::update_connections() {
    std::map<keyframe*, unsigned int> keyfrm_weights;

    std::vector<landmark*> landmarks;
    {
        std::lock_guard<std::mutex> lock(mtx_observations_);
        landmarks = landmarks_;
    }

    for (const auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        const auto observations = lm->get_observations();

        for (const auto& obs : observations) {
            // obs.first: keyframe
            // obs.second: keypoint idx in obs.first
            if (*obs.first == *this) {
                continue;
            }

            // count up weight of obs.first
            keyfrm_weights[obs.first]++;
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    // covisibility graphの閾値
    constexpr unsigned int weight_thr = 15;

    unsigned int max_weight = 0;
    keyframe* nearest_covisibility = nullptr;

    // ソートのためのvectorを作る
    std::vector<std::pair<unsigned int, keyframe*>> weight_keyfrm_pairs;
    weight_keyfrm_pairs.reserve(keyfrm_weights.size());
    for (const auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        // nearest covisibilityの情報を更新
        if (max_weight <= weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }

        // covisibilityの情報を更新
        if (weight_thr < weight) {
            weight_keyfrm_pairs.emplace_back(std::make_pair(weight, keyfrm));
            // 相手側からもグラフを張る
            keyfrm->add_connection(this, weight);
        }
    }

    // 最低一つはweight_keyfrm_pairsに入れておく
    if (weight_keyfrm_pairs.empty()) {
        weight_keyfrm_pairs.emplace_back(std::make_pair(max_weight, nearest_covisibility));
        nearest_covisibility->add_connection(this, max_weight);
    }

    // weightの降順でソート
    std::sort(weight_keyfrm_pairs.rbegin(), weight_keyfrm_pairs.rend());

    // 結果を整形する
    std::vector<keyframe*> ordered_connected_keyfrms;
    ordered_connected_keyfrms.reserve(weight_keyfrm_pairs.size());
    std::vector<unsigned int> ordered_weights;
    ordered_weights.reserve(weight_keyfrm_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_keyfrm_pairs) {
        ordered_connected_keyfrms.push_back(weight_keyfrm_pair.second);
        ordered_weights.push_back(weight_keyfrm_pair.first);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_connections_);

        connected_keyfrms_and_weights_ = keyfrm_weights;
        ordered_connected_keyfrms_ = ordered_connected_keyfrms;
        ordered_weights_ = ordered_weights;

        if (is_first_connection_ && id_ != 0) {
            // nearest covisibilityをspanning treeのparentとする
            assert(*nearest_covisibility == *ordered_connected_keyfrms.front());
            spanning_parent_ = nearest_covisibility;
            spanning_parent_->add_spanning_child(this);
            is_first_connection_ = false;
        }
    }
}

void keyframe::update_covisibility_orders() {
    std::lock_guard<std::mutex> lock(mtx_connections_);

    std::vector<std::pair<unsigned int, keyframe*>> weight_keyfrm_pairs;
    weight_keyfrm_pairs.reserve(connected_keyfrms_and_weights_.size());

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        weight_keyfrm_pairs.emplace_back(std::make_pair(keyfrm_and_weight.second, keyfrm_and_weight.first));
    }

    // weightで降順ソート
    std::sort(weight_keyfrm_pairs.rbegin(), weight_keyfrm_pairs.rend());

    ordered_connected_keyfrms_.clear();
    ordered_connected_keyfrms_.reserve(weight_keyfrm_pairs.size());
    ordered_weights_.clear();
    ordered_weights_.reserve(weight_keyfrm_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_keyfrm_pairs) {
        ordered_connected_keyfrms_.push_back(weight_keyfrm_pair.second);
        ordered_weights_.push_back(weight_keyfrm_pair.first);
    }
}

std::set<keyframe*> keyframe::get_connected_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    std::set<keyframe*> keyfrms;

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        keyfrms.insert(keyfrm_and_weight.first);
    }

    return keyfrms;
}

std::vector<keyframe*> keyframe::get_covisibilities() const {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    return ordered_connected_keyfrms_;
}

std::vector<keyframe*> keyframe::get_top_n_covisibilities(const unsigned int num_covisibilities) const {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    if (ordered_connected_keyfrms_.size() < num_covisibilities) {
        return ordered_connected_keyfrms_;
    }
    else {
        return std::vector<keyframe*>(ordered_connected_keyfrms_.begin(), ordered_connected_keyfrms_.begin() + num_covisibilities);
    }
}

std::vector<keyframe*> keyframe::get_covisibilities_over_weight(const unsigned int weight) const {
    std::lock_guard<std::mutex> lock(mtx_connections_);

    if (ordered_connected_keyfrms_.empty()) {
        return std::vector<keyframe*>();
    }

    auto itr = std::upper_bound(ordered_weights_.begin(), ordered_weights_.end(), weight, std::greater<unsigned int>());
    if (itr == ordered_weights_.end()) {
        return std::vector<keyframe*>();
    }
    else {
        const auto num = static_cast<unsigned int>(itr - ordered_weights_.begin());
        return std::vector<keyframe*>(ordered_connected_keyfrms_.begin(), ordered_connected_keyfrms_.begin() + num);
    }
}

unsigned int keyframe::get_weight(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    if (connected_keyfrms_and_weights_.count(keyfrm)) {
        return connected_keyfrms_and_weights_.at(keyfrm);
    }
    else {
        return 0;
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
    int idx = lm->get_index_in_keyframe(this);
    if (0 <= idx) {
        landmarks_.at(static_cast<unsigned int>(idx)) = nullptr;
    }
}

void keyframe::replace_landmark(const unsigned int idx, landmark* lm) {
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

unsigned int keyframe::get_n_tracked_landmarks(const unsigned int min_num_obs) const {
    std::lock_guard<std::mutex> lock(mtx_observations_);
    unsigned int num_tracked_lms = 0;

    if (0 < min_num_obs) {
        for (const auto lm : landmarks_) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (min_num_obs <= lm->num_observations()) {
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

void keyframe::add_spanning_child(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    spanning_children_.insert(keyfrm);
}

void keyframe::erase_spanning_child(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    spanning_children_.erase(keyfrm);
}

void keyframe::set_spanning_parent(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    assert(!spanning_parent_);
    spanning_parent_ = keyfrm;
}

void keyframe::change_spanning_parent(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    spanning_parent_ = keyfrm;
    keyfrm->add_spanning_child(this);
}

std::set<keyframe*> keyframe::get_spanning_children() const {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    return spanning_children_;
}

keyframe* keyframe::get_spanning_parent() const {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    return spanning_parent_;
}

bool keyframe::has_spanning_child(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    return static_cast<bool>(spanning_children_.count(keyfrm));
}

void keyframe::add_loop_edge(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    // loop edgeになった場合は削除できないようにしておく
    cannot_be_erased_ = true;
    loop_edges_.insert(keyfrm);
}

std::set<keyframe*> keyframe::get_loop_edges() const {
    std::lock_guard<std::mutex> lockCon(mtx_connections_);
    return loop_edges_;
}

void keyframe::set_not_to_be_erased() {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    // 削除できないようにする
    cannot_be_erased_ = true;
}

void keyframe::set_to_be_erased() {
    {
        std::lock_guard<std::mutex> lock(mtx_connections_);
        if (loop_edges_.empty()) {
            cannot_be_erased_ = false;
        }
    }

    if (prepare_for_erasing_) {
        prepare_for_erasing();
    }
}

void keyframe::prepare_for_erasing() {
    {
        std::lock_guard<std::mutex> lock(mtx_connections_);
        if (id_ == 0) {
            // 原点は削除できない
            return;
        }
        else if (cannot_be_erased_) {
            // 削除できないフラグが立っているときは削除しない
            prepare_for_erasing_ = true;
            return;
        }
    }

    // graphのconnectionを削除する
    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        keyfrm_and_weight.first->erase_connection(this);
    }

    // landmarkとのassociationを削除する
    for (const auto lm : landmarks_) {
        if (!lm) {
            continue;
        }
        lm->erase_observation(this);
    }

    {
        std::lock_guard<std::mutex> lock1(mtx_connections_);
        std::lock_guard<std::mutex> lock2(mtx_observations_);

        // connection情報をクリア
        connected_keyfrms_and_weights_.clear();
        ordered_connected_keyfrms_.clear();
        ordered_weights_.clear();

        // 1. 自身のchildrenについて整合性を合わせる処理

        // このkeyframeを削除する場合，
        // このkeyframeをparentとしている他のkeyframeに新たなparentを割り当てる必要がある
        // その候補を探す
        std::set<keyframe*> parent_candidates;
        parent_candidates.insert(spanning_parent_);

        while (!spanning_children_.empty()) {
            bool max_is_found = false;

            unsigned int max_weight = 0;
            keyframe* max_weight_child = nullptr;
            keyframe* max_weight_parent = nullptr;

            for (const auto spanning_child : spanning_children_) {
                // 自分の各childについて調べる
                if (spanning_child->will_be_erased()) {
                    continue;
                }

                // childのconnectionを持ってくる
                auto connections_from_child = spanning_child->get_covisibilities();

                // childとconnectionがあるkeyframes(connection_from_child)と，
                // parentの候補になるkeyframes(parent_candidates)の中で
                //　一致しているものについて， 一番weightが大きいペアを探す
                for (const auto connection_from_child : connections_from_child) {
                    for (const auto& parent_candidate : parent_candidates) {
                        // 一致しているかチェック
                        if (connection_from_child->id_ == parent_candidate->id_) {
                            const auto weight = spanning_child->get_weight(connection_from_child);
                            // maxを比較
                            if (max_weight < weight) {
                                // 更新
                                max_weight = weight;
                                max_weight_child = spanning_child;
                                max_weight_parent = connection_from_child;
                                max_is_found = true;
                            }
                        }
                    }
                }
            }

            if (max_is_found) {
                // ペアが見つかったらspanning treeを更新
                max_weight_child->change_spanning_parent(max_weight_parent);
                parent_candidates.insert(max_weight_child);
                spanning_children_.erase(max_weight_child);
            }
            else {
                // 見つからなかった場合はこれ以上更新できないのでbreak
                break;
            }
        }

        // まだspanning_children_が残っていたら，自身のparentを新たなparentとして割り当てる
        if (!spanning_children_.empty()) {
            for (const auto spanning_child : spanning_children_) {
                spanning_child->change_spanning_parent(spanning_parent_);
            }
        }

        // 2. 自身のparentについて整合性を合わせる処理

        // 削除するだけでOK
        spanning_parent_->erase_spanning_child(this);

        // 3. frame statisticsを更新

        map_db_->replace_reference_keyframe(this, spanning_parent_);

        // 4. 削除フラグを立てる

        will_be_erased_ = true;
    }

    map_db_->erase_keyframe(this);
    bow_db_->erase_keyframe(this);
}

bool keyframe::will_be_erased() {
    std::lock_guard<std::mutex> lock(mtx_connections_);
    return will_be_erased_;
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
        case camera::model_type_t::Fisheye: {
            auto camera = static_cast<camera::fisheye*>(camera_);

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
        case camera::model_type_t::Equirectangular: {
            throw std::runtime_error("Not implemented: Stereo or RGBD of equirectangular camera model");
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

nlohmann::json keyframe::to_json() const {
    // 3次元点IDを取り出す
    std::vector<int> landmark_ids(landmarks_.size(), -1);
    for (unsigned int i = 0; i < landmark_ids.size(); ++i) {
        if (landmarks_.at(i) && !landmarks_.at(i)->will_be_erased()) {
            landmark_ids.at(i) = landmarks_.at(i)->id_;
        }
    }

    // spanning tree childrenを取り出す
    std::vector<int> spanning_child_ids;
    spanning_child_ids.reserve(spanning_children_.size());
    for (const auto spanning_child : spanning_children_) {
        spanning_child_ids.push_back(spanning_child->id_);
    }

    // loop edgesを取り出す
    std::vector<int> loop_edge_ids;
    for (const auto loop_edge : loop_edges_) {
        loop_edge_ids.push_back(loop_edge->id_);
    }

    return {{"src_frm_id", src_frm_id_},
            {"ts", timestamp_},
            {"cam", camera_->name_},
            {"depth_thr", depth_thr_},
            // 姿勢情報
            {"rot_cw", convert_rotation_to_json(cam_pose_cw_.block<3, 3>(0, 0))},
            {"trans_cw", convert_translation_to_json(cam_pose_cw_.block<3, 1>(0, 3))},
            // 特徴点情報
            {"n_keypts", num_keypts_},
            {"keypts", convert_keypoints_to_json(keypts_)},
            {"undists", convert_undistorted_to_json(undist_keypts_)},
            {"x_rights", stereo_x_right_},
            {"depths", depths_},
            {"descs", convert_descriptors_to_json(descriptors_)},
            {"lm_ids", landmark_ids},
            // ORBスケール情報
            {"n_scale_levels", num_scale_levels_},
            {"scale_factor", scale_factor_},
            // グラフ情報
            {"span_parent", spanning_parent_ ? spanning_parent_->id_ : -1},
            {"span_children", spanning_child_ids},
            {"loop_edges", loop_edge_ids}};
}

} // namespace data
} // namespace openvslam
