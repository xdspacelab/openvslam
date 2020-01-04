#include "openvslam/mapping_module.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/module/keyframe_inserter.h"

namespace openvslam {
namespace module {

keyframe_inserter::keyframe_inserter(const camera::setup_type_t setup_type, const float true_depth_thr,
                                     data::map_database* map_db, data::bow_database* bow_db,
                                     const unsigned int min_num_frms, const unsigned int max_num_frms)
    : setup_type_(setup_type), true_depth_thr_(true_depth_thr),
      map_db_(map_db), bow_db_(bow_db),
      min_num_frms_(min_num_frms), max_num_frms_(max_num_frms) {}

void keyframe_inserter::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void keyframe_inserter::reset() {
    frm_id_of_last_keyfrm_ = 0;
}

bool keyframe_inserter::new_keyframe_is_needed(const data::frame& curr_frm, const unsigned int num_tracked_lms,
                                               const data::keyframe& ref_keyfrm) const {
    assert(mapper_);
    // mapping moduleが停止しているときはキーフレームが追加できない
    if (mapper_->is_paused() || mapper_->pause_is_requested()) {
        return false;
    }

    const auto num_keyfrms = map_db_->get_num_keyframes();

    // reference keyframeで観測している3次元点のうち，3視点以上から観測されている3次元点の数を数える
    const unsigned int min_obs_thr = (3 <= num_keyfrms) ? 3 : 2;
    const auto num_reliable_lms = ref_keyfrm.get_num_tracked_landmarks(min_obs_thr);

    // mappingが処理中かどうか
    const bool mapper_is_idle = mapper_->get_keyframe_acceptability();

    // 最新のキーフレームで観測している3次元点数に対する，現在のフレームで観測している3次元点数の割合の閾値
    constexpr unsigned int num_tracked_lms_thr = 15;
    const float lms_ratio_thr = 0.9;

    // 条件A1: 前回のキーフレーム挿入からmax_num_frames_以上経過していたらキーフレームを追加する
    const bool cond_a1 = frm_id_of_last_keyfrm_ + max_num_frms_ <= curr_frm.id_;
    // 条件A2: min_num_frames_以上経過していて,mapping moduleが待機状態であればキーフレームを追加する
    const bool cond_a2 = (frm_id_of_last_keyfrm_ + min_num_frms_ <= curr_frm.id_) && mapper_is_idle;
    // 条件A3: 前回のキーフレームから視点が移動してたらキーフレームを追加する
    const bool cond_a3 = num_tracked_lms < num_reliable_lms * 0.25;

    // 条件B: (キーフレーム追加の必要条件)3次元点が閾値以上観測されていて，3次元点との割合が一定割合以下であればキーフレームを追加する
    const bool cond_b = (num_tracked_lms_thr <= num_tracked_lms) && (num_tracked_lms < num_reliable_lms * lms_ratio_thr);

    // Bが満たされていなければ追加しない
    if (!cond_b) {
        return false;
    }

    // Aのいずれも満たされていなければ追加しない
    if (!cond_a1 && !cond_a2 && !cond_a3) {
        return false;
    }

    if (mapper_is_idle) {
        // mapping moduleが処理中でなければ，とりあえずkeyframeを追加しておく
        return true;
    }

    // mapping moduleが処理中だったら，local BAを止めてキーフレームを追加する
    if (setup_type_ != camera::setup_type_t::Monocular
        && mapper_->get_num_queued_keyframes() <= 2) {
        mapper_->abort_local_BA();
        return true;
    }

    return false;
}

data::keyframe* keyframe_inserter::insert_new_keyframe(data::frame& curr_frm) {
    // mapping moduleを(強制的に)動かす
    if (!mapper_->set_force_to_run(true)) {
        return nullptr;
    }

    curr_frm.update_pose_params();
    auto keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    frm_id_of_last_keyfrm_ = curr_frm.id_;

    // monocularだったらkeyframeをmapping moduleにqueueして終わり
    if (setup_type_ == camera::setup_type_t::Monocular) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // 有効なdepthとそのindexを格納する
    std::vector<std::pair<float, unsigned int>> depth_idx_pairs;
    depth_idx_pairs.reserve(curr_frm.num_keypts_);
    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        const auto depth = curr_frm.depths_.at(idx);
        // depthが有効な範囲のものを追加する
        if (0 < depth) {
            depth_idx_pairs.emplace_back(std::make_pair(depth, idx));
        }
    }

    // 有効なdepthがなかったらqueueして終わり
    if (depth_idx_pairs.empty()) {
        queue_keyframe(keyfrm);
        return keyfrm;
    }

    // カメラに近い順に並べ直す
    std::sort(depth_idx_pairs.begin(), depth_idx_pairs.end());

    // depthを使って3次元点を最小min_num_to_create点作る
    constexpr unsigned int min_num_to_create = 100;
    for (unsigned int count = 0; count < depth_idx_pairs.size(); ++count) {
        const auto depth = depth_idx_pairs.at(count).first;
        const auto idx = depth_idx_pairs.at(count).second;

        // 最小閾値以上の点が追加されて，かつdepthの範囲が敷居を超えたら追加をやめる
        if (min_num_to_create < count && true_depth_thr_ < depth) {
            break;
        }

        // idxに対応する3次元点がある場合はstereo triangulationしない
        {
            auto lm = curr_frm.landmarks_.at(idx);
            if (lm) {
                assert(lm->has_observation());
                continue;
            }
        }


        // idxに対応する3次元がなければstereo triangulationで作る
        const Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        cv::Point2f pt = curr_frm.keypts_[count].pt;
        cv::Vec<uchar, 3> color = curr_frm.img_->at<cv::Vec<uchar, 3>>(pt);
        auto lm = new data::landmark(pos_w, keyfrm, map_db_, color);

        lm->add_observation(keyfrm, idx);
        keyfrm->add_landmark(lm, idx);
        curr_frm.landmarks_.at(idx) = lm;

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
    }

    // keyframeをqueueして終わり
    queue_keyframe(keyfrm);
    return keyfrm;
}

void keyframe_inserter::queue_keyframe(data::keyframe* keyfrm) {
    mapper_->queue_keyframe(keyfrm);
    mapper_->set_force_to_run(false);
}

} // namespace module
} // namespace openvslam
