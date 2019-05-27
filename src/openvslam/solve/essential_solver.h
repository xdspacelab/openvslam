#ifndef OPENVSLAM_SOLVE_ESSENTIAL_SOLVER_H
#define OPENVSLAM_SOLVE_ESSENTIAL_SOLVER_H

#include "openvslam/camera/base.h"
#include "openvslam/util/converter.h"

#include <vector>

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace solve {

class essential_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    template<typename T>
    essential_solver(T* shot_1, T* shot_2, const std::vector<std::pair<int, int>>& matches_12);

    essential_solver(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2,
                     const std::vector<std::pair<int, int>>& matches_12);

    virtual ~essential_solver() = default;

    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    float check_inliers(const Mat33_t& E_21, std::vector<bool>& is_inlier_match);

    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    double get_best_score() const {
        return best_score_;
    }

    Mat33_t get_best_E_21() const {
        return best_E_21_;
    }

    std::vector<bool> get_inlier_matches() const {
        return is_inlier_match_;
    }

    static Mat33_t compute_E_21(const eigen_alloc_vector<Vec3_t>& bearings_1, const eigen_alloc_vector<Vec3_t>& bearings_2);

    static bool decompose(const Mat33_t& E_21, eigen_alloc_vector<Mat33_t>& init_rots, eigen_alloc_vector<Vec3_t>& init_transes);

    template <typename T>
    static Mat33_t create_E_21(T* shot_1, T* shot_2);

private:
    const eigen_alloc_vector<Vec3_t>& bearings_1_;
    const eigen_alloc_vector<Vec3_t>& bearings_2_;
    const std::vector<std::pair<int, int>>& matches_12_;

    bool solution_is_valid_ = false;
    double best_score_ = 0.0;
    Mat33_t best_E_21_;
    std::vector<bool> is_inlier_match_;
};

template<typename T>
essential_solver::essential_solver(T* shot_1, T* shot_2, const std::vector<std::pair<int, int>>& matches_12)
        : bearings_1_(shot_1->bearings_1_), bearings_2_(shot_2->bearings_2_), matches_12_(matches_12) {
    assert(shot_1->camera_->model_type_ == camera::model_type_t::Equirectangular);
    assert(shot_2->camera_->model_type_ == camera::model_type_t::Equirectangular);
}

template <typename T>
Mat33_t essential_solver::create_E_21(T* shot_1, T* shot_2) {
    const Mat33_t rot_1w = shot_1->get_rotation();
    const Vec3_t trans_1w = shot_1->get_translation();
    const Mat33_t rot_2w = shot_2->get_rotation();
    const Vec3_t trans_2w = shot_2->get_translation();

    const Mat33_t rot_21 = rot_2w * rot_1w.transpose();
    const Vec3_t trans_21 = -rot_21 * trans_1w + trans_2w;

    const Mat33_t trans_21_x = util::converter::to_skew_symmetric_mat(trans_21);

    return trans_21_x * rot_21;
}

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_ESSENTIAL_SOLVER_H
