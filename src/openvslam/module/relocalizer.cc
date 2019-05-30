#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/module/relocalizer.h"
#include "openvslam/solve/pnp_solver.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

relocalizer::relocalizer(data::bow_database* bow_db,
                         const double bow_match_lowe_ratio, const double proj_match_lowe_ratio,
                         const unsigned int min_num_bow_matches, const unsigned int min_num_valid_obs)
        : bow_db_(bow_db),
          min_num_bow_matches_(min_num_bow_matches), min_num_valid_obs_(min_num_valid_obs),
          bow_matcher_(bow_match_lowe_ratio, true), proj_matcher_(proj_match_lowe_ratio, true),
          pose_optimizer_() {
    spdlog::debug("CONSTRUCT: module::relocalizer");
}

relocalizer::~relocalizer() {
    spdlog::debug("DESTRUCT: module::relocalizer");
}

bool relocalizer::relocalize(data::frame& curr_frm) {
    curr_frm.compute_bow();

    // relocalizationの候補を持ってくる
    const auto reloc_candidates = bow_db_->acquire_relocalization_candidates(&curr_frm);
    if (reloc_candidates.empty()) {
        return false;
    }
    const auto num_candidates = reloc_candidates.size();

    std::vector<std::unique_ptr<solve::pnp_solver>> pnp_solvers(num_candidates);
    std::vector<std::vector<data::landmark*>> matched_landmarks(num_candidates);
    std::vector<bool> is_discarded(num_candidates, false);

    // 有効なrelocalize候補数
    unsigned int num_valid_candidates = 0;
    // 各候補について，BoW tree matcherで対応点を求める
    for (unsigned int i = 0; i < num_candidates; ++i) {
        auto* keyfrm = reloc_candidates.at(i);
        if (keyfrm->will_be_erased()) {
            is_discarded.at(i) = true;
            continue;
        }

        // マッチングが閾値未満しか得られなければ破棄
        const auto num_matches = bow_matcher_.match_frame_and_keyframe(keyfrm, curr_frm, matched_landmarks.at(i));
        if (num_matches < min_num_bow_matches_) {
            is_discarded.at(i) = true;
            continue;
        }

        pnp_solvers.at(i) = std::unique_ptr<solve::pnp_solver>(new solve::pnp_solver(curr_frm.bearings_, curr_frm.keypts_,
                                                                                       curr_frm.scale_factors_, matched_landmarks.at(i)));
        pnp_solvers.at(i)->set_ransac_parameters(0.99, 10, 300);
        ++num_valid_candidates;
    }

    // 各候補について，
    // 1. PnP(+RANSAC)で姿勢を求める
    // 2. pose optimizerを通す
    // 3. projection matchで対応点を増やす
    // 4. もう一度pose optimizerを通す
    for (unsigned int i = 0; i < num_candidates; ++i) {
        if (is_discarded.at(i)) {
            continue;
        }

        // 1. PnP(+RANSAC)で姿勢を求める

        std::vector<bool> is_inlier;

        if (!pnp_solvers.at(i)->estimate()) {
            continue;
        }

        curr_frm.cam_pose_cw_ = pnp_solvers.at(i)->get_best_cam_pose_cw();
        curr_frm.update_pose_params();

        // 2. pose optimizerを通す

        // EPnPでinlierになった対応のindicesを取得する
        const auto inlier_indices = pnp_solvers.at(i)->get_inlier_indices();
        // 3次元点との対応を再初期化
        curr_frm.landmarks_ = std::vector<data::landmark*>(curr_frm.num_keypts_, nullptr);

        // pose optimizationのために2D-3D対応をセットする
        std::set<data::landmark*> already_found_landmarks;
        for (const auto idx : inlier_indices) {
            // 有効な3次元点のみをcurrent frameにセット
            curr_frm.landmarks_.at(idx) = matched_landmarks.at(i).at(idx);
            // すでに特徴点と対応した3次元点を記録しておく
            already_found_landmarks.insert(matched_landmarks.at(i).at(idx));
        }

        // pose optimization
        auto num_valid_obs = pose_optimizer_.optimize(curr_frm);
        // インライアが閾値未満だったら破棄
        if (num_valid_obs < min_num_bow_matches_ / 2) {
            continue;
        }

        // pose optimizationの際のoutlierを適用する
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; idx++) {
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        // 3. projection matchで対応点を増やす

        // optimizeした姿勢をもとに，projection matchを行う -> 2D-3D対応を設定
        auto num_found = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 10, 100);
        // BoW matchとprojection matchで合わせてインライアが閾値未満だったら破棄
        if (num_valid_obs + num_found < min_num_valid_obs_) {
            continue;
        }

        // 4. もう一度pose optimizerを通す

        // projection matchをもとに，もう一度最適化する
        num_valid_obs = pose_optimizer_.optimize(curr_frm);

        // 閾値未満になったら，もう一度projection matchを行う
        if (num_valid_obs < min_num_valid_obs_) {
            // すでに対応がついているものは除く
            already_found_landmarks.clear();
            for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
                if (!curr_frm.landmarks_.at(idx)) {
                    continue;
                }
                already_found_landmarks.insert(curr_frm.landmarks_.at(idx));
            }
            // もう一度projection matchを行う -> 2D-3D対応を設定
            auto num_additional = proj_matcher_.match_frame_and_keyframe(curr_frm, reloc_candidates.at(i), already_found_landmarks, 3, 64);

            // 閾値未満だったら破棄
            if (num_valid_obs + num_additional < min_num_valid_obs_) {
                continue;
            }

            // もう一度最適化
            num_valid_obs = pose_optimizer_.optimize(curr_frm);

            // 閾値未満だったら破棄
            if (num_valid_obs < min_num_valid_obs_) {
                continue;
            }
        }

        // relocalize成功
        spdlog::info("relocalization succeeded");
        // TODO: current frameのreference keyframeをセットする
        for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
            // アウトライア情報を適用
            if (!curr_frm.outlier_flags_.at(idx)) {
                continue;
            }
            curr_frm.landmarks_.at(idx) = nullptr;
        }

        return true;
    }

    curr_frm.cam_pose_cw_is_valid_ = false;
    return false;
}

} // namespace module
} // namespace openvslam
