#include "openvslam/camera/base.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/match/robust.h"
#include "openvslam/match/angle_checker.h"
#include "openvslam/solve/essential_solver.h"

#ifdef USE_DBOW2
#include <DBoW2/FeatureVector.h>
#else
#include <fbow/fbow.h>
#endif

namespace openvslam {
namespace match {

unsigned int robust::match_for_triangulation(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2, const Mat33_t& E_12,
                                             std::vector<std::pair<unsigned int, unsigned int>>& matched_idx_pairs) {
    unsigned int num_matches = 0;

    angle_checker<int> angle_checker;

    // keyframe2のepipoleの座標を求める = keyframe1のカメラ中心をkeyframe2に投影する
    const Vec3_t cam_center_1 = keyfrm_1->get_cam_center();
    const Mat33_t rot_2w = keyfrm_2->get_rotation();
    const Vec3_t trans_2w = keyfrm_2->get_translation();
    Vec3_t epiplane_in_keyfrm_2;
    keyfrm_2->camera_->reproject_to_bearing(rot_2w, trans_2w, cam_center_1, epiplane_in_keyfrm_2);

    // keyframeの3次元点情報を取得する
    const auto assoc_lms_in_keyfrm_1 = keyfrm_1->get_landmarks();
    const auto assoc_lms_in_keyfrm_2 = keyfrm_2->get_landmarks();

    // matching情報を格納する
    // keyframe1の各特徴点に対して対応を求めるため，keyframe2で既にkeyframe1と対応が取れているものを除外するようにする
    std::vector<bool> is_already_matched_in_keyfrm_2(keyfrm_2->num_keypts_, false);
    // keyframe1のidxと対応しているkeyframe2のidxを格納する
    std::vector<int> matched_indices_2_in_keyfrm_1(keyfrm_1->num_keypts_, -1);

#ifdef USE_DBOW2
    DBoW2::FeatureVector::const_iterator itr_1 = keyfrm_1->bow_feat_vec_.begin();
    DBoW2::FeatureVector::const_iterator itr_2 = keyfrm_2->bow_feat_vec_.begin();
    const DBoW2::FeatureVector::const_iterator itr_1_end = keyfrm_1->bow_feat_vec_.end();
    const DBoW2::FeatureVector::const_iterator itr_2_end = keyfrm_2->bow_feat_vec_.end();
#else
    fbow::BoWFeatVector::const_iterator itr_1 = keyfrm_1->bow_feat_vec_.begin();
    fbow::BoWFeatVector::const_iterator itr_2 = keyfrm_2->bow_feat_vec_.begin();
    const fbow::BoWFeatVector::const_iterator itr_1_end = keyfrm_1->bow_feat_vec_.end();
    const fbow::BoWFeatVector::const_iterator itr_2_end = keyfrm_2->bow_feat_vec_.end();
#endif

    while (itr_1 != itr_1_end && itr_2 != itr_2_end) {
        // BoW treeのノード番号(first)が一致しているか確認する
        if (itr_1->first == itr_2->first) {
            // BoW treeのノード番号(first)が一致していれば，
            // 実際に特徴点index(second)を持ってきて対応しているか確認する
            const auto& keyfrm_1_indices = itr_1->second;
            const auto& keyfrm_2_indices = itr_2->second;

            for (const auto idx_1 : keyfrm_1_indices) {
                auto lm_1 = assoc_lms_in_keyfrm_1.at(idx_1);
                // 3次元点が存在"する"場合はスルー(triangulation前のmatchingであるため)
                if (lm_1) {
                    continue;
                }

                // stereo keypointかどうかをチェックする
                const bool is_stereo_keypt_1 = 0 <= keyfrm_1->stereo_x_right_.at(idx_1);

                // 特徴点・特徴量を取得
                const auto& keypt_1 = keyfrm_1->undist_keypts_.at(idx_1);
                const Vec3_t& bearing_1 = keyfrm_1->bearings_.at(idx_1);
                const auto& desc_1 = keyfrm_1->descriptors_.row(idx_1);

                // keyframe2の特徴点で一番ハミング距離が近いものを探す
                unsigned int best_hamm_dist = HAMMING_DIST_THR_LOW;
                int best_idx_2 = -1;

                for (const auto idx_2 : keyfrm_2_indices) {
                    auto lm_2 = assoc_lms_in_keyfrm_2.at(idx_2);
                    // 3次元点が存在"する"場合はスルー(triangulation前のmatchingであるため)
                    if (lm_2) {
                        continue;
                    }

                    // すでに対応が得られている場合はスルーする
                    if (is_already_matched_in_keyfrm_2.at(idx_2)) {
                        continue;
                    }

                    // stereo keypointかどうかをチェックする
                    const bool is_stereo_keypt_2 = 0 <= keyfrm_2->stereo_x_right_.at(idx_2);

                    // 特徴点・特徴量を取得
                    const Vec3_t& bearing_2 = keyfrm_2->bearings_.at(idx_2);
                    const auto& desc_2 = keyfrm_2->descriptors_.row(idx_2);

                    // 距離計算
                    const auto hamm_dist = compute_descriptor_distance_32(desc_1, desc_2);

                    if (HAMMING_DIST_THR_LOW < hamm_dist || best_hamm_dist < hamm_dist) {
                        continue;
                    }

                    if (!is_stereo_keypt_1 && !is_stereo_keypt_2) {
                        // 両方がstereo keypointでない場合は，エピポール付近の特徴点を使わない
                        const auto cos_dist = epiplane_in_keyfrm_2.dot(bearing_2);
                        // epipoleとbearingの挟角の閾値(=3.0deg)
                        constexpr double cos_dist_thr = 0.99862953475;

                        // 挟角が閾値より小さければmatchさせない
                        if (cos_dist_thr < cos_dist) {
                            continue;
                        }
                    }

                    // E行列による整合性チェック
                    const bool is_inlier = check_epipolar_constraint(bearing_1, bearing_2, E_12,
                                                                     keyfrm_1->scale_factors_.at(keypt_1.octave));
                    if (is_inlier) {
                        best_idx_2 = idx_2;
                        best_hamm_dist = hamm_dist;
                    }
                }

                if (best_idx_2 < 0) {
                    continue;
                }

                is_already_matched_in_keyfrm_2.at(best_idx_2) = true;
                matched_indices_2_in_keyfrm_1.at(idx_1) = best_idx_2;
                ++num_matches;

                if (check_orientation_) {
                    const auto delta_angle
                        = keypt_1.angle - keyfrm_2->undist_keypts_.at(best_idx_2).angle;
                    angle_checker.append_delta_angle(delta_angle, idx_1);
                }
            }

            ++itr_1;
            ++itr_2;
        }
        else if (itr_1->first < itr_2->first) {
            itr_1 = keyfrm_1->bow_feat_vec_.lower_bound(itr_2->first);
        }
        else {
            itr_2 = keyfrm_2->bow_feat_vec_.lower_bound(itr_1->first);
        }
    }

    if (check_orientation_) {
        const auto invalid_matches = angle_checker.get_invalid_matches();
        for (const auto invalid_idx : invalid_matches) {
            matched_indices_2_in_keyfrm_1.at(invalid_idx) = -1;
            --num_matches;
        }
    }

    matched_idx_pairs.clear();
    matched_idx_pairs.reserve(num_matches);

    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_keyfrm_1.size(); ++idx_1) {
        if (matched_indices_2_in_keyfrm_1.at(idx_1) < 0) {
            continue;
        }
        matched_idx_pairs.emplace_back(std::make_pair(idx_1, matched_indices_2_in_keyfrm_1.at(idx_1)));
    }

    return num_matches;
}

unsigned int robust::match_frame_and_keyframe(data::frame& frm, data::keyframe* keyfrm,
                                              std::vector<data::landmark*>& matched_lms_in_frm) {
    // 初期化
    const auto num_frm_keypts = frm.num_keypts_;
    const auto keyfrm_lms = keyfrm->get_landmarks();
    unsigned int num_inlier_matches = 0;
    matched_lms_in_frm = std::vector<data::landmark*>(num_frm_keypts, nullptr);

    // brute-force matchを計算
    std::vector<std::pair<int, int>> matches;
    brute_force_match(frm, keyfrm, matches);

    // eight-point RANSACでインライアのみを抽出
    solve::essential_solver solver(frm.bearings_, keyfrm->bearings_, matches);
    solver.find_via_ransac(50, false);
    if (!solver.solution_is_valid()) {
        return 0;
    }
    const auto is_inlier_matches = solver.get_inlier_matches();

    // 情報を格納する
    for (unsigned int i = 0; i < matches.size(); ++i) {
        if (!is_inlier_matches.at(i)) {
            continue;
        }
        const auto frm_idx = matches.at(i).first;
        const auto keyfrm_idx = matches.at(i).second;

        matched_lms_in_frm.at(frm_idx) = keyfrm_lms.at(keyfrm_idx);
        ++num_inlier_matches;
    }

    return num_inlier_matches;
}

unsigned int robust::brute_force_match(data::frame& frm, data::keyframe* keyfrm, std::vector<std::pair<int, int>>& matches) {
    unsigned int num_matches = 0;

    angle_checker<int> angle_checker;

    // 1. フレームとキーフレームの情報を取得

    const auto num_keypts_1 = frm.num_keypts_;
    const auto num_keypts_2 = keyfrm->num_keypts_;
    const auto keypts_1 = frm.keypts_;
    const auto keypts_2 = keyfrm->keypts_;
    const auto lms_2 = keyfrm->get_landmarks();
    const auto& descs_1 = frm.descriptors_;
    const auto& descs_2 = keyfrm->descriptors_;

    // 2. キーフレームの各descriptorに対して，1番目と2番目に近いフレームのdescriptorを求める
    //    キーフレームのdescriptorは，3次元点と結びついているもののみ対象にする

    // 各idx_1に対して対応しているidx_2
    auto matched_indices_2_in_1 = std::vector<int>(num_keypts_1, -1);
    // 重複を避ける
    std::unordered_set<int> already_matched_indices_1;

    for (unsigned int idx_2 = 0; idx_2 < num_keypts_2; ++idx_2) {
        // 3次元点が有効なもののみ対象にする
        auto lm_2 = lms_2.at(idx_2);
        if (!lm_2) {
            continue;
        }
        if (lm_2->will_be_erased()) {
            continue;
        }

        // キーフレームのdescriptorを取得
        const auto& desc_2 = descs_2.row(idx_2);

        // 1番目と2番目に近いフレームのdescriptorを求める
        unsigned int best_hamm_dist = MAX_HAMMING_DIST;
        int best_idx_1 = -1;
        unsigned int second_best_hamm_dist = MAX_HAMMING_DIST;

        for (unsigned int idx_1 = 0; idx_1 < num_keypts_1; ++idx_1) {
            // 重複を避ける
            if (static_cast<bool>(already_matched_indices_1.count(idx_1))) {
                continue;
            }

            const auto& desc_1 = descs_1.row(idx_1);

            const auto hamm_dist = compute_descriptor_distance_32(desc_2, desc_1);

            if (hamm_dist < best_hamm_dist) {
                second_best_hamm_dist = best_hamm_dist;
                best_hamm_dist = hamm_dist;
                best_idx_1 = idx_1;
            }
            else if (hamm_dist < second_best_hamm_dist) {
                second_best_hamm_dist = hamm_dist;
            }
        }

        if (HAMMING_DIST_THR_LOW < best_hamm_dist) {
            continue;
        }

        if (best_idx_1 < 0) {
            continue;
        }

        // ratio test
        if (lowe_ratio_ * second_best_hamm_dist < static_cast<float>(best_hamm_dist)) {
            continue;
        }

        matched_indices_2_in_1.at(best_idx_1) = idx_2;
        // 重複を避ける
        already_matched_indices_1.insert(best_idx_1);

        if (check_orientation_) {
            const auto delta_angle
                = keypts_1.at(best_idx_1).angle - keypts_2.at(idx_2).angle;
            angle_checker.append_delta_angle(delta_angle, best_idx_1);
        }

        ++num_matches;
    }

    if (check_orientation_) {
        const auto invalid_matches = angle_checker.get_invalid_matches();
        for (const auto invalid_idx_1 : invalid_matches) {
            matched_indices_2_in_1.at(invalid_idx_1) = -1;
            --num_matches;
        }
    }

    matches.clear();
    matches.reserve(num_matches);
    for (unsigned int idx_1 = 0; idx_1 < matched_indices_2_in_1.size(); ++idx_1) {
        const auto idx_2 = matched_indices_2_in_1.at(idx_1);
        if (idx_2 < 0) {
            continue;
        }
        matches.emplace_back(std::make_pair(idx_1, idx_2));
    }

    return num_matches;
}

bool robust::check_epipolar_constraint(const Vec3_t& bearing_1, const Vec3_t& bearing_2,
                                       const Mat33_t& E_12, const float bearing_1_scale_factor) {
    // keyframe1上のtエピポーラ平面の法線ベクトル
    const Vec3_t epiplane_in_1 = E_12 * bearing_2;

    // 法線ベクトルとbearingのなす角を求める
    const auto cos_residual = epiplane_in_1.dot(bearing_1) / epiplane_in_1.norm();
    const auto residual_rad = M_PI / 2.0 - std::abs(std::acos(cos_residual));

    // inlierの閾値(=0.2deg)
    // (e.g. FOV=90deg,横900pixのカメラにおいて,0.2degは横方向の2pixに相当)
    // TODO: 閾値のパラメータ化
    constexpr double residual_deg_thr = 0.2;
    constexpr double residual_rad_thr = residual_deg_thr * M_PI / 180.0;

    // 特徴点スケールが大きいほど閾値を緩くする
    // TODO: thresholdの重み付けの検討
    return residual_rad < residual_rad_thr * bearing_1_scale_factor;
}

} // namespace match
} // namespace openvslam
