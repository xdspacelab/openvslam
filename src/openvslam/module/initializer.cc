#include "openvslam/config.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/initialize/bearing_vector.h"
#include "openvslam/initialize/perspective.h"
#include "openvslam/match/area.h"
#include "openvslam/module/initializer.h"
#include "openvslam/optimize/global_bundle_adjuster.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

initializer::initializer(const std::shared_ptr<config>& cfg, data::map_database* map_db, data::bow_database* bow_db)
        : cfg_(cfg), map_db_(map_db), bow_db_(bow_db) {
    spdlog::debug("CONSTRUCT: module::initializer");
}

initializer::~initializer() {
    spdlog::debug("DESTRUCT: module::initializer");
}

void initializer::reset() {
    delete initializer_;
    initializer_ = nullptr;
    state_ = initializer_state_t::NotReady;
}

initializer_state_t initializer::get_state() const {
    return state_;
}

std::vector<cv::KeyPoint> initializer::get_initial_keypoints() const {
    return init_frm_.keypts_;
}

std::vector<int> initializer::get_initial_matches() const {
    return init_matches_;
}

bool initializer::initialize(data::frame& curr_frm) {
    switch (cfg_->camera_->setup_type_) {
        case camera::setup_type_t::Monocular: {
            // initializerが構築されていない時は構築する
            if (state_ == initializer_state_t::NotReady) {
                create_initializer(curr_frm);
                return false;
            }

            // initializeを試みて，成功したらマップ構築を試行
            if (!try_initialize_for_monocular(curr_frm)) {
                return false;
            }

            // マップ構築が正しく行えたらinitialize成功
            create_map_for_monocular(curr_frm);
            return state_ == initializer_state_t::Succeeded;
        }
        case camera::setup_type_t::Stereo:
        case camera::setup_type_t::RGBD: {
            state_ = initializer_state_t::Initializing;

            // initializeを試みて，成功したらマップ構築を試行
            if (!try_initialize_for_stereo(curr_frm)) {
                return false;
            }

            // マップ構築が正しく行えたらinitialize成功
            create_map_for_stereo(curr_frm);
            return state_ == initializer_state_t::Succeeded;
        }
        default: {
            throw std::runtime_error("Undefined camera setup");
        }
    }
}

void initializer::create_initializer(data::frame& curr_frm) {
    // 初期フレーム
    init_frm_ = data::frame(curr_frm);

    // 初期フレームでの特徴点の位置をセット
    prev_matched_coords_.resize(init_frm_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.undist_keypts_.at(i).pt;
    }

    // 初期フレーム->現在フレームの特徴点の対応を初期化
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // initializerを構築
    delete initializer_;
    initializer_ = nullptr;
    switch (init_frm_.camera_->model_type_) {
        case camera::model_type_t::Perspective:
        case camera::model_type_t::Fisheye: {
            initializer_ = new initialize::perspective(init_frm_);
            break;
        }
        case camera::model_type_t::Equirectangular: {
            initializer_ = new initialize::bearing_vector(init_frm_);
            break;
        }
    }

    state_ = initializer_state_t::Initializing;
}

bool initializer::try_initialize_for_monocular(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    match::area matcher(0.9, true);
    const auto num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);

    if (num_matches < 100) {
        // 次のフレームでinitializerを構築し直す
        reset();
        return false;
    }

    // initializeを試みる
    assert(initializer_);
    return initializer_->initialize(curr_frm, init_matches_);
}

bool initializer::create_map_for_monocular(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    eigen_alloc_vector<Vec3_t> init_triangulated_pts;
    {
        assert(initializer_);
        init_triangulated_pts = initializer_->get_triangulated_pts();
        const auto is_triangulated = initializer_->get_triangulated_flags();

        // triangulationされなかったmatchを無効にする
        for (unsigned int i = 0; i < init_matches_.size(); ++i) {
            if (init_matches_.at(i) < 0) {
                continue;
            }
            if (is_triangulated.at(i)) {
                continue;
            }
            init_matches_.at(i) = -1;
        }

        // 姿勢をセット
        init_frm_.set_cam_pose(Mat44_t::Identity());
        Mat44_t cam_pose_cw = Mat44_t::Identity();
        cam_pose_cw.block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        curr_frm.set_cam_pose(cam_pose_cw);

        delete initializer_;
        initializer_ = nullptr;
    }

    // 初期キーフレームを作成
    auto init_keyfrm = new data::keyframe(init_frm_, map_db_, bow_db_);
    auto curr_keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    init_keyfrm->compute_bow();
    curr_keyfrm->compute_bow();

    // データベースに追加
    map_db_->add_keyframe(init_keyfrm);
    map_db_->add_keyframe(curr_keyfrm);

    // 評価用のframe statisticsを更新
    init_frm_.ref_keyfrm_ = init_keyfrm;
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(init_frm_, false);
    map_db_->update_frame_statistics(curr_frm, false);

    // 3次元点に対して2D-3D対応を追加
    for (unsigned int init_idx = 0; init_idx < init_matches_.size(); init_idx++) {
        const auto curr_idx = init_matches_.at(init_idx);
        if (curr_idx < 0) {
            continue;
        }

        // 3次元点を作成
        auto lm = new data::landmark(init_triangulated_pts.at(init_idx), curr_keyfrm, map_db_);

        init_keyfrm->add_landmark(lm, init_idx);
        curr_keyfrm->add_landmark(lm, curr_idx);

        lm->add_observation(init_keyfrm, init_idx);
        lm->add_observation(curr_keyfrm, curr_idx);

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        // 2D-3D対応を追加
        curr_frm.landmarks_.at(curr_idx) = lm;
        curr_frm.outlier_flags_.at(curr_idx) = false;

        // データベースに追加
        map_db_->add_landmark(lm);
    }

    // global bundle adjustment
    const auto global_bundle_adjuster = optimize::global_bundle_adjuster(map_db_, 20, true);
    global_bundle_adjuster.optimize();

    // depthの中央値が1になるように座標系をスケーリング
    const auto median_depth = init_keyfrm->compute_median_depth(init_keyfrm->camera_->model_type_ == camera::model_type_t::Equirectangular);
    const auto inv_median_depth = 1.0 / median_depth;

    if (curr_keyfrm->get_num_tracked_landmarks(1) < 100 && median_depth < 0) {
        spdlog::info("seems to be wrong initialization, resetting");
        state_ = initializer_state_t::Wrong;
        return false;
    }

    // mapをスケーリング
    scale_map(init_keyfrm, curr_keyfrm, inv_median_depth);
    // 最適化後の姿勢をフレームに再セット
    curr_frm.set_cam_pose(curr_keyfrm->get_cam_pose());

    // 原点を設定
    map_db_->origin_keyfrm_ = init_keyfrm;

    spdlog::info("new map created with {} points: frame {} - frame {}", map_db_->get_num_landmarks(), init_frm_.id_, curr_frm.id_);
    state_ = initializer_state_t::Succeeded;
    return true;
}

void initializer::scale_map(data::keyframe* init_keyfrm, data::keyframe* curr_keyfrm, const double scale) {
    // keyframeをスケーリング
    Mat44_t cam_pose_cw = curr_keyfrm->get_cam_pose();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    curr_keyfrm->set_cam_pose(cam_pose_cw);

    // landmarksをスケーリング
    const auto landmarks = init_keyfrm->get_landmarks();
    for (auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
    }
}

bool initializer::try_initialize_for_stereo(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);
    return 500 <= curr_frm.num_keypts_;
}

bool initializer::create_map_for_stereo(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    // 初期キーフレームを作成
    curr_frm.set_cam_pose(Mat44_t::Identity());
    auto curr_keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    curr_keyfrm->compute_bow();

    // データベースに追加
    map_db_->add_keyframe(curr_keyfrm);

    // 評価用のframe statisticsを更新
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(curr_frm, false);

    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        // stereo triangulationされていたら3次元点を追加する
        const auto z = curr_frm.depths_.at(idx);
        if (z <= 0) {
            continue;
        }

        // 3次元点を作成
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = new data::landmark(pos_w, curr_keyfrm, map_db_);

        // frame,keyframeとlandmarkの対応情報を記録
        lm->add_observation(curr_keyfrm, idx);
        curr_keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        // 3次元点の情報を計算
        lm->compute_descriptor();
        lm->update_normal_and_depth();

        // データベースに追加
        map_db_->add_landmark(lm);
    }

    // 原点を設定
    map_db_->origin_keyfrm_ = curr_keyfrm;

    spdlog::info("new map created with {} points", map_db_->get_num_landmarks());
    state_ = initializer_state_t::Succeeded;
    return true;
}

} // namespace module
} // namespace openvslam
