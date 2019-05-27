#ifndef OPENVSLAM_SOLVE_FUNDAMENTAL_SOLVER_H
#define OPENVSLAM_SOLVE_FUNDAMENTAL_SOLVER_H

#include "openvslam/camera/base.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace solve {

class fundamental_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     * @tparam T data::frame or data::keyframe
     * @param shot_1
     * @param shot_2
     * @param matches_12 matching indices between shot_1 and shot_2
     * @param sigma stddev of symmetric transfer error
     */
    template<typename T>
    fundamental_solver(T* shot_1, T* shot_2, const std::vector<std::pair<int, int>>& matches_12, const float sigma);

    /**
     * Constructor
     * @param undist_keypts_1
     * @param undist_keypts_2
     * @param matches_12 matching indices between shot_1 and shot_2
     * @param sigma stddev of symmetric transfer error
     */
    fundamental_solver(const std::vector<cv::KeyPoint>& undist_keypts_1, const std::vector<cv::KeyPoint>& undist_keypts_2,
                       const std::vector<std::pair<int, int>>& matches_12, const float sigma);

    virtual ~fundamental_solver() = default;

    /**
     * Find fundamental matrix via RANSAC
     * @param max_num_iter
     */
    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    /**
     * Check inliers and compute scores of the specified fundamental matrix
     * @param F_21
     * @param is_inlier_match
     * @return
     */
    float check_inliers(const Mat33_t& F_21, std::vector<bool>& is_inlier_match);

    /**
     * Check if the solution is valid or not
     * @return
     */
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    /**
     * Get best score during RANSAC
     * @return
     */
    double get_best_score() const {
        return best_score_;
    }

    /**
     * Get best model of fundamental transform
     * @return
     */
    Mat33_t get_best_F_21() const {
        return best_F_21_;
    }

    /**
     * Get flags representing inlier matches
     * @return
     */
    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    /**
     * Compute fundamental matrix with minimum set
     * @param keypts_1
     * @param keypts_2
     * @return
     */
    static Mat33_t compute_F_21(const std::vector<cv::Point2f>& keypts_1, const std::vector<cv::Point2f>& keypts_2);

    /**
     * Decompose the specified fundamental matrix into a rotation and a translation
     * @param F_21
     * @param cam_matrix
     * @param init_rots
     * @param init_transes
     * @return
     */
    static bool decompose(const Mat33_t& F_21, const Mat33_t& cam_matrix_1, const Mat33_t& cam_matrix_2,
                          eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes);

private:
    const std::vector<cv::KeyPoint> undist_keypts_1_;
    const std::vector<cv::KeyPoint> undist_keypts_2_;
    const std::vector<std::pair<int, int>>& matches_12_;
    const float sigma_;

    bool solution_is_valid_ = false;
    double best_score_ = 0.0;
    Mat33_t best_F_21_;
    std::vector<bool> is_inlier_match_;
};

template<typename T>
fundamental_solver::fundamental_solver(T* shot_1, T* shot_2, const std::vector<std::pair<int, int>>& matches_12, const float sigma)
        : undist_keypts_1_(shot_1->undist_keypts_), undist_keypts_2_(shot_2->undist_keypts_), matches_12_(matches_12), sigma_(sigma) {
    assert(shot_1->camera_->model_type_ == camera::model_type_t::Perspective);
    assert(shot_2->camera_->model_type_ == camera::model_type_t::Perspective);
}

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_FUNDAMENTAL_SOLVER_H
