#ifndef OPENVSLAM_SOLVE_SIM3_SOLVER_H
#define OPENVSLAM_SOLVE_SIM3_SOLVER_H

#include "openvslam/data/keyframe.h"

#include <vector>
#include <memory>

#include <opencv2/core.hpp>

namespace openvslam {
namespace solve {

class sim3_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    sim3_solver(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                const std::vector<std::shared_ptr<data::landmark>>& matched_lms_in_keyfrm_2,
                const bool fix_scale = true, const unsigned int min_num_inliers = 20);

    //! Destructor
    virtual ~sim3_solver() = default;

    //! Find the most reliable Sim3 matrix via RANSAC
    void find_via_ransac(const unsigned int max_num_iter);

    //! Check if the solution is valid or not
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    //! Get the most reliable rotation from keyframe 2 to keyframe 1
    Mat33_t get_best_rotation_12() {
        return best_rot_12_;
    }

    //! Get the most reliable translation from keyframe 2 to keyframe 1
    Vec3_t get_best_translation_12() {
        return best_trans_12_;
    }

    //! Get the most reliable scale from keyframe 2 to keyframe 1
    float get_best_scale_12() {
        return best_scale_12_;
    }

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
                                  const Mat33_t& rot_21, const Vec3_t& trans_21, const float scale_21, const std::shared_ptr<data::keyframe>& keyfrm);

    //! reproject points in camera (local) coordinates to the same image (as undistorted keypoints)
    void reproject_to_same_image(const std::vector<Vec3_t, Eigen::aligned_allocator<Vec3_t>>& lm_coords_in_cam,
                                 std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>>& reprojected, const std::shared_ptr<data::keyframe>& keyfrm);

protected:
    //! キーフレーム
    std::shared_ptr<data::keyframe> keyfrm_1_;
    std::shared_ptr<data::keyframe> keyfrm_2_;

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

    //! solution is valid or not
    bool solution_is_valid_ = false;
    //! most reliable rotation from keyframe 2 to keyframe 1
    Mat33_t best_rot_12_;
    //! most reliable translation from keyframe 2 to keyframe 1
    Vec3_t best_trans_12_;
    //! most reliable scale from keyframe 2 to keyframe 1
    float best_scale_12_;

    //! common pointsを再投影した画像座標
    //! 3次元点を画像上に再投影した座標(歪みパラメータは適用しない)
    std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>> reprojected_1_;
    std::vector<Vec2_t, Eigen::aligned_allocator<Vec2_t>> reprojected_2_;

    //! RANSACのパラメータ
    unsigned int min_num_inliers_;
};

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_SIM3_SOLVER_H
