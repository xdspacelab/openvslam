#include "openvslam/type.h"
#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/data/landmark.h"
#include "openvslam/map/local_mapper.h"
#include "openvslam/map/loop_closer.h"
#include "openvslam/match/fuse.h"
#include "openvslam/match/robust.h"
#include "openvslam/solver/essential_solver.h"
#include "openvslam/solver/triangulator.h"

#include <unordered_set>
#include <mutex>
#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace map {

local_mapper::local_mapper(data::map_database* map_db, const bool is_monocular)
        : map_db_(map_db), local_bundle_adjuster_(), is_monocular_(is_monocular) {
    spdlog::debug("CONSTRUCT: local_mapper");
}

local_mapper::~local_mapper() {
    spdlog::debug("DESTRUCT: local_mapper");
}

void local_mapper::set_tracker(track::tracker* tracker) {
    tracker_ = tracker;
}

void local_mapper::set_loop_closer(loop_closer* loop_closer) {
    loop_closer_ = loop_closer;
}

void local_mapper::run() {
    spdlog::info("start local mapper");

    is_terminated_ = false;

    while (true) {
        // local mapperの処理中はキーフレームがqueueされないようにする
        set_keyframe_acceptability(false);

        if (keyframe_is_queued()) {
            // queueされたキーフレームをデータベースに保存
            store_new_keyframe();

            // 冗長なlandmarkを削除する
            remove_redundant_landmarks();

            // current keyframeと近傍との対応を使って3次元点を作成する
            create_new_landmarks();

            // 重複する3次元点を統合する
            // (キーフレームがキューされているときはキューを消化してから行う)
            if (!keyframe_is_queued()) {
                update_new_keyframe();
            }

            // BA中にこのフラグが立ったらBAを落とす
            abort_BA_is_requested_ = false;
            // キューを消化していたらlocal BAを行う
            if (!keyframe_is_queued() && !pause_is_requested()) {
                if (2 < map_db_->get_num_keyframes()) {
                    local_bundle_adjuster_.optimize(cur_keyfrm_, &abort_BA_is_requested_);
                }
                // 冗長なキーフレームを削除する
                remove_redundant_keyframes();
            }

            loop_closer_->queue_keyframe(cur_keyfrm_);
        }
        else if (check_and_execute_pause()) {
            // pauseフラグが立っていたらpause処理を行う
            // pauseされるまで待機，pauseされたままterminateされたらlocal mapperを終了する
            while (is_paused() && !check_terminate()) {
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
            }
            if (check_terminate()) {
                break;
            }
        }

        // resetフラグが立っていたらreset処理を行う
        check_and_execute_reset();

        // local mapperの処理が終わったのでキーフレームをqueueできるようにする
        set_keyframe_acceptability(true);

        if (check_terminate()) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // 終了処理を行う
    terminate();
}

void local_mapper::queue_keyframe(data::keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    keyfrms_queue_.push_back(keyfrm);
    abort_BA_is_requested_ = true;
}

unsigned int local_mapper::get_num_queued_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return keyfrms_queue_.size();
}

bool local_mapper::keyframe_is_queued() const {
    std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
    return (!keyfrms_queue_.empty());
}

bool local_mapper::get_keyframe_acceptability() const {
    return keyframe_acceptability_;
}

void local_mapper::set_keyframe_acceptability(const bool acceptability) {
    keyframe_acceptability_ = acceptability;
}

void local_mapper::abort_local_BA() {
    abort_BA_is_requested_ = true;
}

void local_mapper::store_new_keyframe() {
    {
        std::lock_guard<std::mutex> lock(mtx_keyfrm_queue_);
        // queueの先頭を取り出す -> cur_keyfrm_
        cur_keyfrm_ = keyfrms_queue_.front();
        keyfrms_queue_.pop_front();
    }

    const auto cur_lms = cur_keyfrm_->get_landmarks();

    for (unsigned int idx = 0; idx < cur_lms.size(); ++idx) {
        auto lm = cur_lms.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        if (!lm->is_observed_in_keyframe(cur_keyfrm_)) {
            // 3次元点側にキーフレームの情報がなければ追加しておく
            lm->add_observation(cur_keyfrm_, idx);
            lm->update_normal_and_depth();
            lm->compute_descriptor();
        }
        else {
            // 3次元点側にキーフレームの情報が既にあれば，冗長性のチェック対象にする
            fresh_landmarks_.push_back(lm);
        }
    }

    // BoWを計算
    cur_keyfrm_->compute_bow();
    // グラフ情報を更新
    cur_keyfrm_->update_connections();
    // データベースに保存
    map_db_->add_keyframe(cur_keyfrm_);
}

void local_mapper::create_new_landmarks() {
    // キーフレームとその近傍キーフレームの間でtriangulationする
    const auto cur_covisibilities = cur_keyfrm_->get_top_n_covisibilities(is_monocular_ ? 20 : 10);

    // lowe_ratio will not be used
    match::robust matcher(0.0, false);

    // currentのカメラ姿勢
    const Vec3_t cur_cam_center = cur_keyfrm_->get_cam_center();

    for (unsigned int i = 0; i < cur_covisibilities.size(); ++i) {
        if (1 < i && keyframe_is_queued()) {
            return;
        }

        // neighbor keyframe
        auto ngh_keyfrm = cur_covisibilities.at(i);

        // neighborのカメラ姿勢
        const Vec3_t ngh_cam_center = ngh_keyfrm->get_cam_center();

        // currentとneighborのベースライン長を求めて，triangulationするかどうかを判定する
        const Vec3_t baseline_vec = ngh_cam_center - cur_cam_center;
        const auto baseline_dist = baseline_vec.norm();
        if (is_monocular_) {
            // シーンに比べて極端にベースラインのスケールが小さい場合はtriangulationしない
            const float median_depth_in_ngh = ngh_keyfrm->compute_median_depth(true);
            if (baseline_dist / median_depth_in_ngh < 0.01) {
                continue;
            }
        }
        else {
            // stereoの場合，stereoのベースライン以上離れていなければtriangulationしない
            if (baseline_dist < ngh_keyfrm->camera_->true_baseline_) {
                continue;
            }
        }

        // currentとneighborの間でマッチングを求め，E行列に基づいたアウトライア除去を行う
        // (cur bearing) * E_ngh_to_cur * (ngh bearing) = 0
        const Mat33_t E_ngh_to_cur = solver::essential_solver::create_E_21(ngh_keyfrm, cur_keyfrm_);
        std::vector<std::pair<unsigned int, unsigned int>> matches;
        matcher.match_for_triangulation(cur_keyfrm_, ngh_keyfrm, E_ngh_to_cur, matches);

        // triangulationを行う
        triangulate_with_two_keyframes(cur_keyfrm_, ngh_keyfrm, matches);
    }
}

void local_mapper::triangulate_with_two_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                                  const std::vector<std::pair<unsigned int, unsigned int>>& matches) {
    // currentのカメラ姿勢
    const Mat33_t rot_1w = keyfrm_1->get_rotation();
    const Mat33_t rot_w1 = rot_1w.transpose();
    const Vec3_t trans_1w = keyfrm_1->get_translation();
    const Mat44_t cam_pose_1w = keyfrm_1->get_cam_pose();
    const Vec3_t cam_center_1 = keyfrm_1->get_cam_center();
    const auto camera_1 = keyfrm_1->camera_;

    // neighborのカメラ姿勢
    const Mat33_t rot_2w = keyfrm_2->get_rotation();
    const Mat33_t rot_w2 = rot_2w.transpose();
    const Vec3_t trans_2w = keyfrm_2->get_translation();
    const Mat44_t cam_pose_2w = keyfrm_2->get_cam_pose();
    const Vec3_t cam_center_2 = keyfrm_2->get_cam_center();
    const auto camera_2 = keyfrm_2->camera_;

    const float ratio_factor = 1.6f * cur_keyfrm_->scale_factor_;

#ifdef _OPENMP
#pragma omp parallel for
#endif
    for (unsigned int j = 0; j < matches.size(); ++j) {
        const auto idx_1 = matches.at(j).first;
        const auto idx_2 = matches.at(j).second;

        const auto& keypt_1 = keyfrm_1->undist_keypts_.at(idx_1);
        const float keypt_1_x_right = keyfrm_1->stereo_x_right_.at(idx_1);
        const bool is_stereo_1 = 0 <= keypt_1_x_right;

        const auto& keypt_2 = keyfrm_2->undist_keypts_.at(idx_2);
        const float keypt_2_x_right = keyfrm_2->stereo_x_right_.at(idx_2);
        const bool is_stereo_2 = 0 <= keypt_2_x_right;

        // 各特徴点のbearing vectorを求める
        const Vec3_t ray_c_1 = keyfrm_1->bearings_.at(idx_1);
        const Vec3_t ray_c_2 = keyfrm_2->bearings_.at(idx_2);

        const Vec3_t ray_w_1 = rot_w1 * ray_c_1;
        const Vec3_t ray_w_2 = rot_w2 * ray_c_2;
        const auto cos_rays_parallax = ray_w_1.dot(ray_w_2);

        // ステレオカメラである場合は3次元点をステレオ視した際のcos parallax([0, 1])を計算
        const auto cos_stereo_parallax_1 = is_stereo_1 ? std::cos(2.0 * atan2(camera_1->true_baseline_ / 2.0, keyfrm_1->depths_.at(idx_1))) : 2.0;
        const auto cos_stereo_parallax_2 = is_stereo_2 ? std::cos(2.0 * atan2(camera_2->true_baseline_ / 2.0, keyfrm_2->depths_.at(idx_2))) : 2.0;

        const auto cos_stereo_parallax = std::min(cos_stereo_parallax_1, cos_stereo_parallax_2);

        // linear triangulationを行う条件
        // 視差角の閾値(=1.0deg)
        constexpr double cos_rays_parallax_thr = 0.99984769515;
        const bool linear_triangulation =
                ((!is_stereo_1 && !is_stereo_2) && 0.0 < cos_rays_parallax && cos_rays_parallax < cos_rays_parallax_thr) // monocularで十分視差角が確保できる場合
                || ((is_stereo_1 || is_stereo_2) && 0.0 < cos_rays_parallax && cos_rays_parallax < cos_stereo_parallax); // stereoよりも別視点とのほうが視差角が確保できる場合

        Vec3_t pos_w;
        if (linear_triangulation) {
            pos_w = solver::triangulator::triangulate(ray_c_1, ray_c_2, cam_pose_1w, cam_pose_2w);
        }
        else if (is_stereo_1 && cos_stereo_parallax_1 < cos_stereo_parallax_2) {
            pos_w = keyfrm_1->triangulate_stereo(idx_1);
        }
        else if (is_stereo_2 && cos_stereo_parallax_2 < cos_stereo_parallax_1) {
            pos_w = keyfrm_2->triangulate_stereo(idx_2);
        }
        else {
            continue;
        }

        // 3次元点がカメラの前方にあることを確認

        if (!check_depth_is_positive(pos_w, rot_1w, trans_1w, camera_1)
            || !check_depth_is_positive(pos_w, rot_2w, trans_2w, camera_2)) {
            continue;
        }

        // 再投影誤差が大きい物を除く

        if (!check_reprojection_error(pos_w, rot_1w, trans_1w, camera_1, keypt_1.pt, keypt_1_x_right,
                                      keyfrm_1->level_sigma_sq_.at(keypt_1.octave), is_stereo_1)
            || !check_reprojection_error(pos_w, rot_2w, trans_2w, camera_2, keypt_2.pt, keypt_2_x_right,
                                         keyfrm_2->level_sigma_sq_.at(keypt_2.octave), is_stereo_2)) {
            continue;
        }

        // 3次元点とカメラの間の距離から求めたスケール比と，特徴点のスケール比が大きく異なる場合は破棄

        if (!check_scale_factors(pos_w, cam_center_1, cam_center_2, ratio_factor,
                                 keyfrm_1->scale_factors_.at(keypt_1.octave),
                                 keyfrm_2->scale_factors_.at(keypt_2.octave))) {
            continue;
        }

        // triangulationに成功
        auto lm = new data::landmark(pos_w, keyfrm_1, map_db_);

        lm->add_observation(keyfrm_1, idx_1);
        lm->add_observation(keyfrm_2, idx_2);

        keyfrm_1->add_landmark(lm, idx_1);
        keyfrm_2->add_landmark(lm, idx_2);

        lm->compute_descriptor();
        lm->update_normal_and_depth();

        map_db_->add_landmark(lm);
#ifdef _OPENMP
#pragma omp critical
#endif
        {
            fresh_landmarks_.push_back(lm);
        }
    }
}

void local_mapper::remove_redundant_landmarks() {
    constexpr float found_per_visible_ratio_thr = 0.25;
    constexpr unsigned int num_reliable_keyfrms = 2;
    const unsigned int num_obs_thr = is_monocular_ ? 2 : 3;

    auto iter = fresh_landmarks_.begin();
    const unsigned int cur_keyfrm_id = cur_keyfrm_->id_;

    while (iter != fresh_landmarks_.end()) {
        auto lm = *iter;
        if (lm->will_be_erased()) {
            // 3次元点が削除されたらバッファから削除
            iter = fresh_landmarks_.erase(iter);
        }
        else if (lm->get_found_per_visible_ratio() < found_per_visible_ratio_thr) {
            // 3次元点があまり観測されなかった場合は
            // 3次元点を削除する処理をしてからバッファから削除
            lm->prepare_for_erasing();
            iter = fresh_landmarks_.erase(iter);
        }
        else if (num_reliable_keyfrms + lm->first_keyfrm_id_ <= cur_keyfrm_id
                 && lm->num_observations() <= num_obs_thr) {
            // キーフレームがいくつか挿入された後でも3次元点を観測しているキーフレームが閾値以下だった場合は
            // 3次元点を削除する処理をしてからバッファから削除
            lm->prepare_for_erasing();
            iter = fresh_landmarks_.erase(iter);
        }
        else if (num_reliable_keyfrms + 1U + lm->first_keyfrm_id_ <= cur_keyfrm_id) {
            // キーフレームがいくつか挿入されたあとであれば
            // 3次元点が有効であるとしてバッファから削除
            iter = fresh_landmarks_.erase(iter);
        }
        else {
            // 次のキーフレーム挿入時に再判断する
            iter++;
        }
    }
}

void local_mapper::remove_redundant_keyframes() {
    // 削除対象としないウィンドウの大きさ
    constexpr unsigned int window_size_not_to_remove = 2;
    // 削除対象になる冗長な観測の割合の閾値
    constexpr float redundant_obs_ratio_thr = 0.9;

    // currentキーフレームのグラフ上の近傍キーフレーム各々に対して冗長かどうかの判定を行う
    const auto cur_covisibilities = cur_keyfrm_->get_covisibilities();
    for (const auto covisibility : cur_covisibilities) {
        // 起点のキーフレームは削除しない
        if (*covisibility == *(map_db_->origin_keyfrm_)) {
            continue;
        }
        // 最近追加されたキーフレームは削除しない
        if (covisibility->id_ <= cur_keyfrm_->id_
            && cur_keyfrm_->id_ <= covisibility->id_ + window_size_not_to_remove) {
            continue;
        }

        // covisibilityについて，冗長な観測の数(num_redundant_obs)を数える
        unsigned int num_redundant_obs = 0;
        unsigned int num_valid_obs = 0;
        count_redundant_landmarks(covisibility, num_valid_obs, num_redundant_obs);

        // covisibilityの観測のうち，冗長なものの割合が90%を超えていたらkeyfrmを削除する
        if (redundant_obs_ratio_thr <= static_cast<float>(num_redundant_obs) / num_valid_obs) {
            covisibility->prepare_for_erasing();
        }
    }
}

void local_mapper::count_redundant_landmarks(data::keyframe* keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const {
    // 自身よりも小さいスケールで同じ3次元点を観測しているキーフレームが閾値以上あった場合，
    // 自身によるその3次元点の観測は冗長であると判断する
    constexpr unsigned int num_better_obs_thr = 3;

    num_valid_obs = 0;
    num_redundant_obs = 0;

    // keyfrmが観測している3次元点各々(lm)について，
    // 自身以外の観測キーフレーム(ngh_keyfrm)がどの特徴点スケールで3次元点を観測しているかを調べる
    const auto landmarks = keyfrm->get_landmarks();

    for (unsigned int idx = 0; idx < landmarks.size(); idx++) {
        auto lm = landmarks.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // stereoカメラモデルで，depthが無効な(=キーフレーム間でtriangulationした)場合は冗長でないと判断
        const auto depth = keyfrm->depths_.at(idx);
        if (!is_monocular_ && (depth < 0.0 || keyfrm->depth_thr_ < depth)) {
            continue;
        }

        ++num_valid_obs;

        // 3次元点(lm)を観測しているキーフレーム数が閾値(num_obs_thr)以下なら冗長でないと判断
        if (lm->num_observations() <= num_better_obs_thr) {
            continue;
        }

        // keyfrmでは3次元点lmを特徴点スケールscale_levelで観測している
        const auto scale_level = keyfrm->undist_keypts_.at(idx).octave;
        // 3次元点lmを見ているキーフレームの情報
        const auto observations = lm->get_observations();

        // 冗長な観測であることを表すフラグ
        bool is_redundant = false;

        // 3次元点lmに対して，keyfrmよりもlmを近くで観測しているキーフレーム数
        unsigned int num_better_obs = 0;

        for (const auto obs : observations) {
            const auto ngh_keyfrm = obs.first;
            if (*ngh_keyfrm == *keyfrm) {
                continue;
            }

            // ngh_keyfrmでは3次元点lmを特徴点スケールngh_scale_levelで観測している
            const auto ngh_scale_level = ngh_keyfrm->undist_keypts_.at(obs.second).octave;

            // keyfrmの観測とngh_keyfrmの観測のスケールを比較
            if (ngh_scale_level - 1 <= scale_level) {
                // ngh_keyfrmのほうがより近くで3次元点を観測している
                ++num_better_obs;
                if (num_better_obs_thr <= num_better_obs) {
                    // 閾値を超えていたら，keyfrmによるこのobservationは冗長であると判断
                    is_redundant = true;
                    break;
                }
            }
        }
        if (is_redundant) {
            // keyfrmが観測している3次元点のうち，冗長と判断されたものの数を数えておく
            ++num_redundant_obs;
        }
    }
}

void local_mapper::update_new_keyframe() {
    // 2次の近傍キーフレームまで取得する
    const auto fuse_tgt_keyfrms = get_second_order_covisibilities(is_monocular_ ? 20 : 10, 5);

    // currentキーフレームと近傍キーフレームとの間で3次元点の重複を解消する
    fuse_landmark_duplication(fuse_tgt_keyfrms);

    // 各3次元点のobservation情報を更新する
    const auto cur_landmarks = cur_keyfrm_->get_landmarks();
    for (const auto lm : cur_landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }
        lm->compute_descriptor();
        lm->update_normal_and_depth();
    }

    // グラフ情報を更新する
    cur_keyfrm_->update_connections();
}

std::unordered_set<data::keyframe*> local_mapper::get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                                  const unsigned int second_order_thr) {
    // 近傍のキーフレームを取得する
    const auto cur_covisibilities = cur_keyfrm_->get_top_n_covisibilities(first_order_thr);

    // currentのキーフレームとマッチングを取るキーフレームを入れておく集合
    std::unordered_set<data::keyframe*> fuse_tgt_keyfrms;
    fuse_tgt_keyfrms.reserve(cur_covisibilities.size() * 2);

    for (const auto keyfrm : cur_covisibilities) {
        if (keyfrm->will_be_erased()) {
            continue;
        }

        // 重複を避ける(setなので重複しないけど)
        if (static_cast<bool>(fuse_tgt_keyfrms.count(keyfrm))) {
            continue;
        }
        fuse_tgt_keyfrms.insert(keyfrm);

        // 近傍キーフレームのさらに近傍をたどる
        const auto ngh_covisibilities = keyfrm->get_top_n_covisibilities(second_order_thr);
        for (const auto ngh_keyfrm : ngh_covisibilities) {
            if (ngh_keyfrm->will_be_erased()) {
                continue;
            }
            // 近傍の近傍には自分自身が入るので除外する
            if (*ngh_keyfrm == *cur_keyfrm_) {
                continue;
            }

            // 重複を避ける(setなので重複しないけど)
            if (static_cast<bool>(fuse_tgt_keyfrms.count(ngh_keyfrm))) {
                continue;
            }
            fuse_tgt_keyfrms.insert(ngh_keyfrm);
        }
    }

    return fuse_tgt_keyfrms;
}

void local_mapper::fuse_landmark_duplication(const std::unordered_set<data::keyframe*>& fuse_tgt_keyfrms) {
    match::fuse matcher;

    {
        // currentのキーフレームで観測している3次元点をtargetの各キーフレームに投影して
        // - さらなる対応点
        // - 対応点の重複
        // を探して追加・修正する

        auto cur_landmarks = cur_keyfrm_->get_landmarks();
        for (const auto fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            matcher.replace_duplication(fuse_tgt_keyfrm, cur_landmarks);
        }
    }

    {
        // targetの各キーフレームで観測している3次元点をcurrentのキーフレームに投影して
        // - さらなる対応点
        // - 対応点の重複
        // を探して追加・修正する

        std::unordered_set<data::landmark*> candidate_landmarks_to_fuse;
        candidate_landmarks_to_fuse.reserve(fuse_tgt_keyfrms.size() * cur_keyfrm_->num_keypts_);

        for (const auto fuse_tgt_keyfrm : fuse_tgt_keyfrms) {
            const auto fuse_tgt_landmarks = fuse_tgt_keyfrm->get_landmarks();

            for (const auto lm : fuse_tgt_landmarks) {
                if (!lm) {
                    continue;
                }
                if (lm->will_be_erased()) {
                    continue;
                }

                // 重複を避ける(setなので重複しないけど)
                if (static_cast<bool>(candidate_landmarks_to_fuse.count(lm))) {
                    continue;
                }
                candidate_landmarks_to_fuse.insert(lm);
            }
        }

        matcher.replace_duplication(cur_keyfrm_, candidate_landmarks_to_fuse);
    }
}

void local_mapper::request_reset() {
    {
        std::lock_guard<std::mutex> lock(mtx_reset_);
        reset_is_requested_ = true;
    }

    // resetされるまで(reset_is_requested_フラグが落とされるまで)待機する
    while (true) {
        {
            std::lock_guard<std::mutex> lock(mtx_reset_);
            if (!reset_is_requested_) {
                spdlog::info("reset local mapper");
                break;
            }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(3000));
    }
}

void local_mapper::check_and_execute_reset() {
    // 排他制御をしてフラグを確認
    std::lock_guard<std::mutex> lock(mtx_reset_);
    if (!reset_is_requested_) {
        return;
    }
    // リセットフラグが立っていたらリセットする
    keyfrms_queue_.clear();
    fresh_landmarks_.clear();
    reset_is_requested_ = false;
}

void local_mapper::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
    std::lock_guard<std::mutex> lock2(mtx_keyfrm_queue_);
    abort_BA_is_requested_ = true;
}

bool local_mapper::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool local_mapper::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

bool local_mapper::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_ && !force_to_run_) {
        is_paused_ = true;
        spdlog::info("pause local mapper");
        return true;
    }
    else {
        return false;
    }
}

bool local_mapper::set_force_to_run(const bool force_to_run) {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    if (force_to_run && is_paused_) {
        return false;
    }

    force_to_run_ = force_to_run;
    return true;
}

void local_mapper::resume() {
    // pauseとterminateのフラグを触るので排他制御
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    std::lock_guard<std::mutex> lock2(mtx_terminate_);

    // 既にlocal mapperが停止している場合は再開できない
    if (is_terminated_) {
        return;
    }

    // 再開する処理を行う
    is_paused_ = false;
    pause_is_requested_ = false;

    // TODO: 最後のキーフレームの追加処理を行う
    for (auto& new_keyframe : keyfrms_queue_) {
        delete new_keyframe;
    }

    keyfrms_queue_.clear();

    spdlog::info("resume local mapper");
}

void local_mapper::request_terminate() {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    terminate_is_requested_ = true;
}

bool local_mapper::is_terminated() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return is_terminated_;
}

bool local_mapper::check_terminate() const {
    std::lock_guard<std::mutex> lock(mtx_terminate_);
    return terminate_is_requested_;
}

void local_mapper::terminate() {
    std::lock_guard<std::mutex> lock1(mtx_terminate_);
    is_terminated_ = true;
    std::lock_guard<std::mutex> lock2(mtx_pause_);
    is_paused_ = true;
}

} // namespace map
} // namespace openvslam
