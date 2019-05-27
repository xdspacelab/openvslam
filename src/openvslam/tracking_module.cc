#include "openvslam/config.h"
#include "openvslam/system.h"
#include "openvslam/tracking_module.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/feature/orb_extractor.h"
#include "openvslam/match/projection.h"
#include "openvslam/util/image_converter.h"

#include <chrono>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace openvslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
        : cfg_(cfg), camera_(cfg->camera_), system_(system), map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
          initializer_(cfg, map_db, bow_db), frame_tracker_(camera_, 10), relocalizer_(bow_db_), pose_optimizer_(),
          keyfrm_inserter_(cfg_->camera_->setup_type_, cfg_->true_depth_thr_, map_db, bow_db, 0, cfg_->camera_->fps_) {
    spdlog::debug("CONSTRUCT: tracking_module");

    extractor_left_ = new feature::orb_extractor(cfg_->orb_params_);
    if (camera_->setup_type_ == camera::setup_type_t::Monocular) {
        ini_extractor_left_ = new feature::orb_extractor(cfg_->orb_params_);
        ini_extractor_left_->set_max_num_keypoints(ini_extractor_left_->get_max_num_keypoints() * 2);
    }
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(cfg_->orb_params_);
    }
}

tracking_module::~tracking_module() {
    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete ini_extractor_left_;
    ini_extractor_left_ = nullptr;

    spdlog::debug("DESTRUCT: tracking_module");
}

void tracking_module::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
    keyfrm_inserter_.set_mapping_module(mapper);
}

void tracking_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

std::vector<cv::KeyPoint> tracking_module::get_initial_keypoints() const {
    return initializer_.get_initial_keypoints();
}

std::vector<int> tracking_module::get_initial_matches() const {
    return initializer_.get_initial_matches();
}

Mat44_t tracking_module::track_monocular_image(const cv::Mat& img, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = img;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);

    // create current frame object
    if (tracking_state_ == tracker_state_t::NotInitialized || tracking_state_ == tracker_state_t::Initializing) {
        curr_frm_ = data::frame(img_gray_, timestamp, ini_extractor_left_, bow_vocab_, camera_, cfg_->true_depth_thr_, mask);
    }
    else {
        curr_frm_ = data::frame(img_gray_, timestamp, extractor_left_, bow_vocab_, camera_, cfg_->true_depth_thr_, mask);
    }

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

Mat44_t tracking_module::track_stereo_image(const cv::Mat& left_img_rect, const cv::Mat& right_img_rect, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = left_img_rect;
    cv::Mat right_img_gray = right_img_rect;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    // create current frame object
    curr_frm_ = data::frame(img_gray_, right_img_gray, timestamp, extractor_left_, extractor_right_, bow_vocab_, camera_, cfg_->true_depth_thr_, mask);

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

Mat44_t tracking_module::track_RGBD_image(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask) {
    const auto start = std::chrono::system_clock::now();

    // color and depth scale conversion
    img_gray_ = img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_true_depth(img_depth, cfg_->depthmap_factor_);

    // create current frame object
    curr_frm_ = data::frame(img_gray_, img_depth, timestamp, extractor_left_, bow_vocab_, camera_, cfg_->true_depth_thr_, mask);

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

void tracking_module::reset() {
    spdlog::info("resetting system");

    initializer_.reset();
    keyfrm_inserter_.reset();

    mapper_->request_reset();
    global_optimizer_->request_reset();

    bow_db_->clear();
    map_db_->clear();

    data::frame::next_id_ = 0;
    data::keyframe::next_id_ = 0;
    data::landmark::next_id_ = 0;

    last_reloc_frm_id_ = 0;

    tracking_state_ = tracker_state_t::NotInitialized;
}

void tracking_module::track() {
    if (tracking_state_ == tracker_state_t::NotInitialized) {
        tracking_state_ = tracker_state_t::Initializing;
    }

    last_tracking_state_ = tracking_state_;

    // check if pause is requested
    check_and_execute_pause();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // fix map mutex
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    if (tracking_state_ == tracker_state_t::Initializing) {
        if (!initialize()) {
            return;
        }

        // reference keyframeとlocal keyframesとlocal landmarksを更新
        update_local_map();

        // mapping moduleにkeyframesを渡しておく
        const auto keyfrms = map_db_->get_all_keyframes();
        for (const auto keyfrm : keyfrms) {
            mapper_->queue_keyframe(keyfrm);
        }

        // tracking modeに移行
        tracking_state_ = tracker_state_t::Tracking;
    }
    else {
        // 3次元点にreplaceを適用する
        apply_landmark_replace();
        // last frameの姿勢を更新(local BAでreference keyframeが更新されている可能性があるので)
        update_last_frame();

        bool succeeded;
        if (mapping_is_enabled_) {
            // mapping mode
            succeeded = track_current_frame();
        }
        else {
            // localization mode;
            succeeded = localize_current_frame();
        }

        curr_frm_.ref_keyfrm_ = ref_keyfrm_;

        // local mapの情報を更新
        if (succeeded) {
            succeeded = track_local_map();
        }

        // 評価用のframe statisticsを更新
        map_db_->update_frame_statistics(curr_frm_, !succeeded);

        // 状態遷移
        tracking_state_ = succeeded ? tracker_state_t::Tracking : tracker_state_t::Lost;

        // 1秒以内にlostしたらリセットする
        if (tracking_state_ == tracker_state_t::Lost && curr_frm_.id_ < camera_->fps_) {
            spdlog::info("tracking lost soon after initialization");
            system_->request_reset();
            return;
        }

        // lostしていたらメッセージを表示する
        if (last_tracking_state_ != tracker_state_t::Lost && tracking_state_ == tracker_state_t::Lost) {
            spdlog::info("tracking lost");
        }

        // キーフレームを追加するか判断する
        if (succeeded && new_keyframe_is_needed()) {
            insert_new_keyframe();
        }

        // current frameのtrackingが成功している場合はmotion modelを更新しておく
        if (succeeded) {
            // 等速運動モデルを更新
            update_motion_model();

            // BAでアウトライアにされたものは排除する(keyframeを追加した後に行う必要がある)
            for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
                if (curr_frm_.landmarks_.at(idx) && curr_frm_.outlier_flags_.at(idx)) {
                    curr_frm_.landmarks_.at(idx) = nullptr;
                }
            }
        }
    }

    // last frameを更新する
    last_frm_ = curr_frm_;

    // ref_keyfrm_からの相対姿勢を保持しておく
    // (次回のtracking開始時に，ref_keyfrm_の姿勢が更新されていた際にlast frameの位置の更新を行うため)
    if (curr_frm_.cam_pose_cw_is_valid_) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.cam_pose_cw_ * curr_frm_.ref_keyfrm_->get_cam_pose_inv();
    }
}

bool tracking_module::initialize() {
    initializer_.initialize(curr_frm_);

    // map構築に失敗(=Wrong)していたらreset
    if (initializer_.get_state() == module::initializer_state_t::Wrong) {
        reset();
        return false;
    }

    // initializeに失敗していたら(!=Succeeded)，次のフレームでinitializeを再試行するのでreturn
    if (initializer_.get_state() != module::initializer_state_t::Succeeded) {
        return false;
    }

    return true;
}

bool tracking_module::track_current_frame() {
    bool succeeded;
    if (tracking_state_ == tracker_state_t::Tracking) {
        // trackingが成功していれば，通常のtrackingを続ける
        if (velocity_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
            succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, velocity_);
            if (!succeeded) {
                succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
                if (!succeeded) {
                    succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
                }
            }
        }
        else {
            succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            if (!succeeded) {
                succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            }
        }
    }
    else {
        // trackingがlostしていれば，relocalizeを試みる
        succeeded = relocalizer_.relocalize(curr_frm_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
        }
    }
    return succeeded;
}

bool tracking_module::localize_current_frame() {
    bool succeeded;
    if (tracking_state_ == tracker_state_t::Lost) {
        // trackingがlostしていれば，relocalizeを試みる
        succeeded = relocalizer_.relocalize(curr_frm_);
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
        }
    }
    else {
        if (velocity_is_valid_) {
            succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, velocity_);
            if (!succeeded) {
                succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
                if (!succeeded) {
                    succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
                }
            }
        }
        else {
            succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            if (!succeeded) {
                succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            }
        }
    }
    return succeeded;
}

void tracking_module::update_motion_model() {
    if (last_frm_.cam_pose_cw_is_valid_) {
        Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
        last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rotation_inv();
        last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_cam_center();
        velocity_is_valid_ = true;
        velocity_ = curr_frm_.cam_pose_cw_ * last_frm_cam_pose_wc;
    }
    else {
        velocity_is_valid_ = false;
        velocity_ = Mat44_t::Identity();
    }
}

void tracking_module::apply_landmark_replace() {
    for (unsigned int idx = 0; idx < last_frm_.num_keypts_; ++idx) {
        auto lm = last_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        auto replaced_lm = lm->get_replaced();
        if (replaced_lm) {
            last_frm_.landmarks_.at(idx) = replaced_lm;
        }
    }
}

void tracking_module::update_last_frame() {
    // local map optimizationによってkeyframeの姿勢が変わっている可能性があるので，それに応じてlast frameの姿勢を更新
    auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
    if (!last_ref_keyfrm) {
        return;
    }
    last_frm_.set_cam_pose(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_cam_pose());
}

bool tracking_module::track_local_map() {
    // reference keyframeとlocal keyframesとlocal landmarksを更新
    update_local_map();

    // local landmarksをcurrent frameに再投影して2D-3D対応を求める
    search_local_landmarks();

    // pose optimizationを走らせる
    pose_optimizer_.optimize(&curr_frm_);

    // 3次元点と対応している特徴点の数をカウントする
    num_tracked_lms_ = 0;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        if (!curr_frm_.outlier_flags_.at(idx)) {
            // BAでインライアだった場合
            assert(lm->has_observation());
            // 3次元点との対応が得られたのでカウンタを増やす
            lm->increase_num_found();
            ++num_tracked_lms_;
        }
        else {
            // BAでアウトライアだった場合
            // 3次元点との対応を削除する
            curr_frm_.landmarks_.at(idx) = nullptr;
        }
    }

    // 3次元点と対応している特徴点の数の閾値
    constexpr unsigned int num_tracked_lms_thr = 20;

    // relocalizeしてすぐは閾値を厳しくする
    if (curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_ && num_tracked_lms_ < 2 * num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, 2 * num_tracked_lms_thr);
        return false;
    }

    // 閾値を超えていればtracking成功
    if (num_tracked_lms_ < num_tracked_lms_thr) {
        spdlog::debug("local map tracking failed: {} matches < {}", num_tracked_lms_, num_tracked_lms_thr);
        return false;
    }
    else {
        return true;
    }
}

void tracking_module::update_local_map() {
    update_local_keyframes();
    update_local_landmarks();

    // 可視化用
    map_db_->set_local_landmarks(local_landmarks_);
}

void tracking_module::update_local_keyframes() {
    // current frameと隣接しているkeyframeについて，共有している3次元点の数を数える
    std::unordered_map<data::keyframe*, unsigned int> keyfrm_weights;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            curr_frm_.landmarks_.at(idx) = nullptr;
        }

        const auto observations = lm->get_observations();
        for (auto obs : observations) {
            ++keyfrm_weights[obs.first];
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    unsigned int max_weight = 0;
    data::keyframe* nearest_covisibility = nullptr;

    local_keyfrms_.clear();
    local_keyfrms_.reserve(3 * keyfrm_weights.size());

    // 3次元点を共有しているキーフレームを集計する
    for (auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        if (keyfrm->will_be_erased()) {
            continue;
        }

        local_keyfrms_.push_back(keyfrm);
        keyfrm->ref_frm_id_in_tracking_ = curr_frm_.id_;

        // nearest covisibilityを求める
        if (weight > max_weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }
    }

    // local keyframesと隣接しているものと更に隣接しているものを追加する
    for (std::vector<data::keyframe*>::const_iterator iter = local_keyfrms_.begin(), end = local_keyfrms_.end(); iter != end; ++iter) {
        // TODO: ここの閾値を増やし過ぎるとtrackingが落ちやすくなる
        if (local_keyfrms_.size() > 80) {
            break;
        }

        auto keyfrm = *iter;

        // 隣接keyframe
        const auto neighbors = keyfrm->get_top_n_covisibilities(10);
        for (auto neighbor : neighbors) {
            if (neighbor->will_be_erased()) {
                continue;
            }
            // 重複を避ける
            if (neighbor->ref_frm_id_in_tracking_ == curr_frm_.id_) {
                continue;
            }

            local_keyfrms_.push_back(neighbor);
            neighbor->ref_frm_id_in_tracking_ = curr_frm_.id_;
            break;
        }

        // spanning treeのchildren
        const auto spanning_children = keyfrm->get_spanning_children();
        for (auto child : spanning_children) {
            if (child->will_be_erased()) {
                continue;
            }
            // 重複を避ける
            if (child->ref_frm_id_in_tracking_ == curr_frm_.id_) {
                continue;
            }

            local_keyfrms_.push_back(child);
            child->ref_frm_id_in_tracking_ = curr_frm_.id_;
            break;
        }

        // spanning treeのparent
        auto parent = keyfrm->get_spanning_parent();
        if (!parent) {
            continue;
        }
        // 重複を避ける
        if (parent->ref_frm_id_in_tracking_ == curr_frm_.id_) {
            continue;
        }

        local_keyfrms_.push_back(parent);
        parent->ref_frm_id_in_tracking_ = curr_frm_.id_;
    }

    if (nearest_covisibility) {
        ref_keyfrm_ = nearest_covisibility;
        curr_frm_.ref_keyfrm_ = ref_keyfrm_;
    }
}

void tracking_module::update_local_landmarks() {
    local_landmarks_.clear();

    for (auto keyfrm : local_keyfrms_) {
        const auto lms = keyfrm->get_landmarks();

        for (auto lm : lms) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }
            if (lm->ref_frm_id_in_tracking_ == curr_frm_.id_) {
                continue;
            }

            local_landmarks_.push_back(lm);
            lm->ref_frm_id_in_tracking_ = curr_frm_.id_;
        }
    }
}

void tracking_module::search_local_landmarks() {
    // current frameとすでに対応すしている3次元点は再投影する必要がないのでマークをつける
    for (auto& lm : curr_frm_.landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            lm = nullptr;
            continue;
        }

        lm->increase_num_visible();
        lm->last_frm_id_in_tracking_ = curr_frm_.id_;
        lm->is_observed_in_tracking_ = false;
    }

    unsigned int num_lms_to_reproj = 0;

    for (auto lm : local_landmarks_) {
        // マークが付いているものはスルー
        if (lm->last_frm_id_in_tracking_ == curr_frm_.id_) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // 再投影して可観測かどうかをチェックする
        // & 再投影したときの再投影点をframeに登録する
        if (curr_frm_.is_observable(lm, 0.5)) {
            lm->increase_num_visible();
            ++num_lms_to_reproj;
        }
    }

    if (num_lms_to_reproj == 0) {
        return;
    }

    // 再投影点の情報を使って2D(current frame)-3D(local landmarks)のマッチングを得る
    match::projection projection_matcher(0.8);
    const float margin = (curr_frm_.id_ < last_reloc_frm_id_ + 2)
                         ? 20.0 : ((camera_->setup_type_ == camera::setup_type_t::RGBD)
                                   ? 10.0 : 5.0);
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, margin);
}

bool tracking_module::new_keyframe_is_needed() const {
    if (!mapping_is_enabled_) {
        return false;
    }

    // relocalizeしてから1秒以上経過していなければキーフレームを追加しない
    const auto num_keyfrms = map_db_->get_num_keyframes();
    if (cfg_->camera_->fps_ < num_keyfrms && curr_frm_.id_ < last_reloc_frm_id_ + cfg_->camera_->fps_) {
        return false;
    }

    // keyframe inserterでキーフレームとしての妥当性をチェック
    return keyfrm_inserter_.new_keyframe_is_needed(curr_frm_, num_tracked_lms_, *ref_keyfrm_);
}

void tracking_module::insert_new_keyframe() {
    // keyframe inserterでキーフレームを挿入
    const auto ref_keyfrm = keyfrm_inserter_.insert_new_keyframe(curr_frm_);
    // 挿入されたキーフレームのポインタをreference keyframeにセットする
    ref_keyfrm_ = ref_keyfrm ? ref_keyfrm : ref_keyfrm_;
    curr_frm_.ref_keyfrm_ = ref_keyfrm_;
}

void tracking_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
}

bool tracking_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool tracking_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void tracking_module::resume() {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    // 再開する処理を行う
    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume tracker");
}

bool tracking_module::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause tracker");
        return true;
    }
    else {
        return false;
    }
}

} // namespace openvslam
