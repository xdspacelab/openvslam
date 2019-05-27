#ifndef OPENVSLAM_SOLVE_SIM3_SOLVER_H
#define OPENVSLAM_SOLVE_SIM3_SOLVER_H

#include "openvslam/data/keyframe.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace solve {

class sim3_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    sim3_solver() = delete;

    sim3_solver(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                const std::vector<data::landmark*>& matched_lms_in_keyfrm_2, const bool fix_scale = true);

    virtual ~sim3_solver() = default;

    void set_ransac_parameters(const float probability = 0.99,
                               const unsigned int min_num_inliers = 6,
                               const unsigned int max_num_iterations = 500);

    bool estimate();

    //! get best inlier index pairs of landmarks
    std::vector<std::pair<unsigned int, unsigned int>> get_inlier_indices() {
        std::vector<std::pair<unsigned int, unsigned int>> inlier_indices;
        inlier_indices.reserve(num_common_pts_);

        for (unsigned int i = 0; i < num_common_pts_; ++i) {
            if (!best_inliers_.at(i)) {
                continue;
            }
            const auto idx1 = matched_indices_1_.at(i);
            const auto idx2 = matched_indices_2_.at(i);
            inlier_indices.emplace_back(std::make_pair(idx1, idx2));
        }

        return inlier_indices;
    }

    //! get best rotation from keyframe2 to keyframe1
    Mat33_t get_best_rotation_12() { return best_rot_12_; }

    //! get best translation from keyframe2 to keyframe1
    Vec3_t get_best_translation_12() { return best_trans_12_; }

    //! get best scale from keyframe2 to keyframe1
    float get_best_scale_12() { return best_scale_12_; }

protected:
    //! compute Sim3 from three common points
    //! points1(3点)の座標系をpoints2(3点)に変換する相似変換行列を推定する
    //! (入力行列は，各列が [x_i, y_i, z_i].T であり，計3列が行方向に並んでいる)
    void compute_Sim3(const Mat33_t& pts_1, const Mat33_t& pts_2,
                      Mat33_t& rot_12, Vec3_t& trans_12, float& scale_12,
                      Mat33_t& rot_21, Vec3_t& trans_21, float& scale_21);

    //! count up inliers
    unsigned int count_inliers(const Mat33_t& rot_12, const Vec3_t& trans_12, const float scale_12,
                               const Mat33_t& rot_21, const Vec3_t& trans_21, const float scale_21,
                               std::vector<bool>& inliers);

    //! reproject points in camera (local) coordinates to the other image (as undistorted keypoints)
    void reproject_to_other_image(const std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& lm_coords_in_cam_1,
                                  std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>>& reprojected_in_cam_2,
                                  const Mat33_t& rot_21, const Vec3_t& trans_21, const float scale_21, data::keyframe* keyfrm);

    //! reproject points in camera (local) coordinates to the same image (as undistorted keypoints)
    void reproject_to_same_image(const std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& lm_coords_in_cam,
                                 std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>>& reprojected, data::keyframe* keyfrm);

protected:
    //! キーフレーム
    data::keyframe* keyfrm_1_;
    data::keyframe* keyfrm_2_;

    //! true: Sim3を求める, false: SE3を求める
    bool fix_scale_;

    // コンストラクタ内で計算する変数
    //! common associated points in keyframe1 and keyframe2
    //! keyframe1とkeyframe2で共通の3次元点座標を，それぞれのキーフレームのローカル座標に変換して保存しておく
    std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>> common_pts_in_keyfrm_1_;
    std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>> common_pts_in_keyfrm_2_;
    //! 自由度2のカイ2乗値x再投影誤差分散
    std::vector<float> chi_sq_x_sigma_sq_1_;
    std::vector<float> chi_sq_x_sigma_sq_2_;
    //! 共通の3次元点座標を出す際の各キーフレームの特徴点index
    std::vector<size_t> matched_indices_1_;
    std::vector<size_t> matched_indices_2_;
    //! 共通3次元点数
    unsigned int num_common_pts_ = 0;

    //! RANSACで得られたベストモデル
    std::vector<bool> best_inliers_;
    Mat33_t best_rot_12_;
    Vec3_t best_trans_12_;
    float best_scale_12_;

    //! common pointsを再投影した画像座標
    //! 3次元点を画像上に再投影した座標(歪みパラメータは適用しない)
    std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>> reprojected_1_;
    std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>> reprojected_2_;

    //! RANSACのパラメータ
    float probability_;
    unsigned int min_num_inliers_;
    unsigned int max_num_iterations_;
};

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_SIM3_SOLVER_H
