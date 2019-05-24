#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/map/loop_detector.h"
#include "openvslam/match/bow_tree.h"
#include "openvslam/match/projection.h"
#include "openvslam/solver/sim3_solver.h"
#include "openvslam/util/converter.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace map {

loop_detector::loop_detector(data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale)
        : bow_db_(bow_db), bow_vocab_(bow_vocab), transform_optimizer_(fix_scale), fix_scale_(fix_scale) {}

bool loop_detector::detect_loop_candidates(data::keyframe* keyfrm) {
    cur_keyfrm_ = keyfrm;
    // loop detectorが停止されていたらループ検出を行わない
    // or ループ修正を行ってからすぐはループ修正を行わない
    if (!loop_detector_is_enabled_ || keyfrm->id_ < prev_loop_correct_keyfrm_id_ + 10) {
        // このキーフレームでは何もしないのでデータベースに追加する
        bow_db_->add_keyframe(keyfrm);
        // 削除を許可
        keyfrm->set_to_be_erased();
        return false;
    }

    // 1. BoWデータベースに問い合わせてループ候補を探す

    // 1-1. 近傍キーフレームで，BoWの類似度の最小値を計算する -> 探索の際の最小スコアにする

    const float min_score = compute_min_score_in_covisibilities(keyfrm);

    // 1-2. min_scoreを使ってBoWデータベースに類似キーフレームを問い合わせる

    const auto init_loop_candidates = bow_db_->acquire_loop_candidates(keyfrm, min_score);

    // 1-3. 候補がなかったらループ修正を行わない

    if (init_loop_candidates.empty()) {
        // 連続してキーフレーム集合が検出されなかったのでバッファをclearする
        cont_detected_keyfrm_sets_.clear();
        // このキーフレームでは何もしないのでデータベースに追加する
        bow_db_->add_keyframe(keyfrm);
        // 削除を許可
        keyfrm->set_to_be_erased();
        return false;
    }

    // 2. init_loop_candidatesの各ループ候補を，単一フレームから近傍キーフレームに拡げる
    //    各キーフレーム集合が何回連続して検出されたかを記録していく
    //    (前回同じキーフレーム集合が検出されていれば，cont_detected_keyfrm_sets_に一致するキーフレーム集合が存在する)
    //    (キーフレーム集合が一致している = 2つの集合の積集合が空集合でないこと)

    const auto curr_cont_detected_keyfrm_sets =
            find_continuously_detected_keyframe_sets(cont_detected_keyfrm_sets_, init_loop_candidates);

    // 3. 閾値(min_continuity_)以上連続して検出されたキーフレーム集合を新たなループ候補として採用する

    loop_candidates_to_validate_.clear();

    for (auto& curr : curr_cont_detected_keyfrm_sets) {
        const auto candidate_keyfrm = curr.lead_keyfrm_;
        const auto continuity = curr.continuity_;
        if (min_continuity_ <= continuity) {
            loop_candidates_to_validate_.push_back(candidate_keyfrm);
        }
    }

    // 4. 次回に備えてメンバ変数を更新

    // 連続してループ検出されたキーフレーム集合と連続回数を更新
    cont_detected_keyfrm_sets_ = curr_cont_detected_keyfrm_sets;

    // ループ検出が終了したのでBoWデータベースに追加
    bow_db_->add_keyframe(keyfrm);

    if (loop_candidates_to_validate_.empty()) {
        // ループ候補がなければ現在のキーフレームの削除を許可する
        keyfrm->set_to_be_erased();
        return false;
    }
    else {
        // 引き続きループ修正を行うので，現在のキーフレームの削除は禁止
        return true;
    }
}

bool loop_detector::validate_candidates() {
    // ループ候補の削除を禁止する
    for (const auto candidate : loop_candidates_to_validate_) {
        candidate->set_not_to_be_erased();
    }

    // 1. ループ候補の各キーフレームに対し，観測されている3次元点を使って相似変換行列を求め，Sim3の条件を満たされているものを探す

    const bool candidate_is_found = select_loop_candidate_via_Sim3(loop_candidates_to_validate_, final_candidate_keyfrm_,
                                                                   g2o_Sim3_world_to_curr_, curr_assoc_lms_in_cand_);
    Sim3_world_to_curr_ = util::converter::to_eigen_mat(g2o_Sim3_world_to_curr_);

    if (!candidate_is_found) {
        for (auto& loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        cur_keyfrm_->set_to_be_erased();
        return false;
    }

    spdlog::debug("detect loop candidate via Sim3 estimation: keyframe {} - keyframe {}", final_candidate_keyfrm_->id_, cur_keyfrm_->id_);

    // 2. 得られた相似変換行列を使って，ループ候補周辺の3次元点をcurrentのキーフレームに再投影し，
    //    さらに2D-3D対応を追加する

    // candidateとcandidate周辺で見えている3次元点と，currentの特徴点との対応情報
    curr_assoc_lms_near_cand_.clear();

    auto cand_covisibilities = final_candidate_keyfrm_->get_covisibilities();
    cand_covisibilities.push_back(final_candidate_keyfrm_);

    for (const auto covisibility : cand_covisibilities) {
        const auto lms_in_covisibility = covisibility->get_landmarks();
        for (const auto lm : lms_in_covisibility) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            if (lm->keyfrm_id_in_loop_validation_ == cur_keyfrm_->id_) {
                continue;
            }
            curr_assoc_lms_near_cand_.push_back(lm);
            lm->keyfrm_id_in_loop_validation_ = cur_keyfrm_->id_;
        }
    }

    // candidate周辺の3次元点(curr_assoc_lms_near_cand_)をSim3(Sim3_world_to_curr_)を用いて
    // current keyframeに再投影し，2次元点-3次元点の対応(curr_assoc_lms_in_cand_)を追加する
    // curr_assoc_lms_in_cand_ですでに対応が取られている3次元点は再投影する必要が無いので除外する
    match::projection projection_matcher(0.75, true);
    projection_matcher.match_by_Sim3_transform(cur_keyfrm_, Sim3_world_to_curr_, curr_assoc_lms_near_cand_, curr_assoc_lms_in_cand_, 10);

    // 最終的なマッチング数を数える
    unsigned int num_final_matches = 0;
    for (const auto curr_assoc_lm_in_cand : curr_assoc_lms_in_cand_) {
        if (curr_assoc_lm_in_cand) {
            ++num_final_matches;
        }
    }

    spdlog::debug("acquired {} matches after projection-match", num_final_matches);

    constexpr unsigned int num_final_matches_thr = 40;
    if (num_final_matches_thr <= num_final_matches) {
        // loop-closeを実行する
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            // 最終的なループ候補以外の削除を許可する
            if (*loop_candidate == *final_candidate_keyfrm_) {
                continue;
            }
            loop_candidate->set_to_be_erased();
        }
        return true;
    }
    else {
        // loop-closeを実行しない
        spdlog::debug("destruct loop candidate because enough matches not acquired (< {})", num_final_matches_thr);
        // ループ候補全ての削除を許可する
        for (const auto loop_candidate : loop_candidates_to_validate_) {
            loop_candidate->set_to_be_erased();
        }
        cur_keyfrm_->set_to_be_erased();
        return false;
    }
}

bool loop_detector::select_loop_candidate_via_Sim3(const std::vector<data::keyframe*>& loop_candidates,
                                                 data::keyframe*& selected_candidate,
                                                 g2o::Sim3& g2o_Sim3_world_to_curr,
                                                 std::vector<data::landmark*>& curr_assoc_lms_in_cand) const {
    // ループ候補の各キーフレームに対し，観測されている3次元点を使って相似変換行列を求める
    // 相似変換行列は，線形方程式のRANSAC -> 非線形最適化の順に推定し，
    // 都度，インライア数が閾値を下回っていたらそのループ候補は破棄する

    match::bow_tree bow_matcher(0.75, true);
    match::projection projection_matcher(0.75, true);

    for (const auto candidate : loop_candidates) {
        // 削除対象であれば破棄
        if (candidate->will_be_erased()) {
            continue;
        }

        // BoWツリーで対応点を探索する
        // TODO: BF-matchに切り替える
        // cur_keyfrm_とcandidateの間で対応点を探索し，candidateで観測されている3次元点との対応を得る
        // (Sim3の計算に，currentで観測されている3次元点とcandidateで観測されている3次元点の対応が必要なため)
        std::vector<data::landmark*> assoc_lms_in_cand;
        const auto num_matches = bow_matcher.match_keyframes(cur_keyfrm_, candidate, assoc_lms_in_cand);
        // 対応数が閾値より少なかったら破棄
        if (num_matches < 20) {
            continue;
        }

        // 対応点を使ってSim3を求める
        // keyframe1: current keyframe, keyframe2: candidate keyframe
        // 2->1(candidate->current)のSim3を求める
        solver::sim3_solver solver(cur_keyfrm_, candidate, assoc_lms_in_cand, fix_scale_);
        solver.set_ransac_parameters(0.999, 20, 300);
        const auto estimated = solver.estimate();
        if (!estimated) {
            continue;
        }

        spdlog::debug("found loop candidate via linear Sim3 estimation: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        const Mat33_t rot_cand_to_curr = solver.get_best_rotation_12();
        const Vec3_t trans_cand_to_curr = solver.get_best_translation_12();
        const float scale_cand_to_curr = solver.get_best_scale_12();

        projection_matcher.match_keyframes_mutually(cur_keyfrm_, candidate, assoc_lms_in_cand, scale_cand_to_curr, rot_cand_to_curr, trans_cand_to_curr, 7.5);

        g2o::Sim3 g2o_sim3_cand_to_curr(rot_cand_to_curr, trans_cand_to_curr, scale_cand_to_curr);
        const auto num_optimized_inliers = transform_optimizer_.optimize(cur_keyfrm_, candidate, assoc_lms_in_cand, g2o_sim3_cand_to_curr, 10);

        // インライア数が閾値より少なかったら破棄
        if (num_optimized_inliers < 20) {
            continue;
        }

        spdlog::debug("found loop candidate via nonlinear Sim3 optimization: keyframe {} - keyframe {}", candidate->id_, cur_keyfrm_->id_);

        selected_candidate = candidate;
        g2o_Sim3_world_to_curr = g2o_sim3_cand_to_curr * g2o::Sim3(candidate->get_rotation(), candidate->get_translation(), 1.0);
        curr_assoc_lms_in_cand = assoc_lms_in_cand;

        return true;
    }

    return false;
}

float loop_detector::compute_min_score_in_covisibilities(data::keyframe* keyfrm) const {
    // スコアの最大値は1.0
    float min_score = 1.0;

    // キーフレームとそのcovisibilityの間で類似度を計算して，最小値を探索する
    const auto covisibilities = keyfrm->get_covisibilities();
    const auto& bow_vec_1 = keyfrm->bow_vec_;
    for (const auto covisibility : covisibilities) {
        if (covisibility->will_be_erased()) {
            continue;
        }
        const auto& bow_vec_2 = covisibility->bow_vec_;
        // currentとcovisibilityの間で類似度を計算して，最小値を探索する
#ifdef USE_DBOW2
        const float score = bow_vocab_->score(bow_vec_1, bow_vec_2);
#else
        const float score = fbow::BoWVector::score(bow_vec_1, bow_vec_2);
#endif
        if (score < min_score) {
            min_score = score;
        }
    }

    return min_score;
}

keyframe_sets loop_detector::find_continuously_detected_keyframe_sets(const keyframe_sets& prev_cont_detected_keyfrm_sets,
                                                                      const std::vector<data::keyframe*>& keyfrms_to_search) const {
    // 各キーフレーム集合が何回連続して検出されたかを数える

    // 連続してループ検出されたキーフレーム集合と回数を保存しておく
    std::vector<keyframe_set> curr_cont_detected_keyfrm_sets;

    // 集合同士が全単射になるように制限をかけるためのフラグ集合
    // prev_cont_detected_keyfrm_setsの各キーフレーム集合に対応するキーフレーム集合がただ1つになるようにする
    std::map<std::set<data::keyframe*>, bool> already_matched;
    for (const auto& prev : prev_cont_detected_keyfrm_sets) {
        already_matched[prev.keyfrm_set_] = false;
    }
    // これ以降はalready_matched.at()でアクセスする(constant access)

    for (const auto& loop_candidate : keyfrms_to_search) {
        // 各ループ候補を，単一フレームから近傍キーフレーム集合に拡げる -> curr_keyfrm_set
        auto curr_keyfrm_set = loop_candidate->get_connected_keyframes();

        // first_candidate_setがいずれとも一致しなかったら初期化するためにフラグを設定しておく
        bool initialization_is_needed = true;

        // 前回のループ検出までに連続して検出されているキーフレーム集合と比較して，一致するか確認
        for (const auto& prev : prev_cont_detected_keyfrm_sets) {
            // prev.keyfrm_set_: キーフレームの集合
            // prev.lead_keyfrm_: 中心のキーフレーム
            // prev.continuity_: 連続検出回数

            // 前回ループ候補として検出されたキーフレーム集合(prev.keyfrm_set_)が
            // 既に今回ループ候補として検出されたキーフレーム集合のうちどれかと一致するとされていたらスルーする
            if (already_matched.at(prev.keyfrm_set_)) {
                continue;
            }

            // cand_keyfrm_setとprev.keyfrm_set_の
            // 積集合が空集合でなければ一致しているとする
            if (prev.intersection_is_empty(curr_keyfrm_set)) {
                continue;
            }

            // curr_keyfrm_setがprev_cont_detected_keyfrm_setsのどれかと一致したので，
            // curr_keyfrm_setを新たに追加する必要はない
            initialization_is_needed = false;

            // curr_keyfrm_setが連続してcurr_continuity回検出された -> 集計
            const auto curr_continuity = prev.continuity_ + 1;
            curr_cont_detected_keyfrm_sets.emplace_back(
                    keyframe_set{curr_keyfrm_set, loop_candidate, curr_continuity});

            // cont_detected_keyfrm_setがprev.keyfrm_set_と一致したのでフラグを立てる
            already_matched.at(prev.keyfrm_set_) = true;
        }

        // 前回のキーフレーム集合に対し，curr_keyfrm_setがどれとも一致しなかったらそれを新たに追加しておく
        if (initialization_is_needed) {
            curr_cont_detected_keyfrm_sets.emplace_back(
                    keyframe_set{curr_keyfrm_set, loop_candidate, 0});
        }
    }

    return curr_cont_detected_keyfrm_sets;
}

} // namespace map
} // namespace openvslam
