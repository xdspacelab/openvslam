#ifndef OPENVSLAM_SOLVE_PNP_SOLVER_H
#define OPENVSLAM_SOLVE_PNP_SOLVER_H

#include "openvslam/util/converter.h"

#include <vector>

namespace openvslam {
namespace solve {

class pnp_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    pnp_solver(const eigen_alloc_vector<Vec3_t>& valid_bearings, const std::vector<cv::KeyPoint>& valid_keypts,
               const eigen_alloc_vector<Vec3_t>& valid_landmarks, const std::vector<float>& scale_factors,
               const unsigned int min_num_inliers = 10);

    //! Destructor
    virtual ~pnp_solver();

    //! Find the most reliable camera pose via RANSAC
    void find_via_ransac(const unsigned int max_num_iter, const bool recompute = true);

    //! Check if the solution is valid or not
    bool solution_is_valid() const {
        return solution_is_valid_;
    }

    //! Get the most reliable rotation (as the world reference)
    Mat33_t get_best_rotation() const {
        return best_rot_cw_;
    }

    //! Get the most reliable translation (as the world reference)
    Vec3_t get_best_translation() const {
        return best_trans_cw_;
    }

    //! Get the most reliable camera pose (as the world reference)
    Mat44_t get_best_cam_pose() const {
        return util::converter::to_eigen_cam_pose(best_rot_cw_, best_trans_cw_);
    }

    //! Get the inlier flags estimated via RANSAC
    std::vector<bool> get_inlier_flags() const {
        return is_inlier_match;
    }

private:
    //! Check inliers of 2D-3D matches
    //! (Note: inlier flags are set to_inlier_match and the number of inliers is returned)
    unsigned int check_inliers(const Mat33_t& rot_cw, const Vec3_t& trans_cw, std::vector<bool>& is_inlier);

    //! the number of 2D-3D matches
    const unsigned int num_matches_;
    // the following vectors are corresponded as element-wise
    //! bearing vector
    eigen_alloc_vector<Vec3_t> valid_bearings_;
    //! 3D point
    eigen_alloc_vector<Vec3_t> valid_landmarks_;
    //! acceptable maximum error
    std::vector<float> max_cos_errors_;

    //! minimum number of inliers
    //! (Note: if the number of inliers is less than this, solution is regarded as invalid)
    const unsigned int min_num_inliers_;

    //! solution is valid or not
    bool solution_is_valid_ = false;
    //! most reliable rotation
    Mat33_t best_rot_cw_;
    //! most reliable translation
    Vec3_t best_trans_cw_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match;

    //-----------------------------------------
    // quoted from EPnP implementation

    void reset_correspondences();

    void set_max_num_correspondences(const unsigned int max_num_correspondences);

    void add_correspondence(const Vec3_t& pos_w, const Vec3_t& bearing);

    double compute_pose(Mat33_t& rot_cw, Vec3_t& trans_cw);

    double reprojection_error(const double R[3][3], const double t[3]);

    void choose_control_points();

    void compute_barycentric_coordinates();

    void fill_M(MatX_t& M, const int row, const double* alphas, const double u, const double v);

    void compute_ccs(const double* betas, const MatX_t& ut);

    void compute_pcs();

    void solve_for_sign();

    void find_betas_approx_1(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas);

    void find_betas_approx_2(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas);

    void find_betas_approx_3(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas);

    void qr_solve(MatRC_t<6, 4>& A, MatRC_t<6, 1>& b, MatRC_t<4, 1>& X);

    double dot(const double* v1, const double* v2);

    double dist2(const double* p1, const double* p2);

    void compute_rho(MatRC_t<6, 1>& Rho);

    void compute_L_6x10(const MatX_t& Ut, MatRC_t<6, 10>& L_6x10);

    void gauss_newton(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double current_betas[4]);

    void compute_A_and_b_gauss_newton(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double cb[4], MatRC_t<6, 4>& A, MatRC_t<6, 1>& b);

    double compute_R_and_t(const MatX_t& Ut, const double* betas, double R[3][3], double t[3]);

    void estimate_R_and_t(double R[3][3], double t[3]);

    double* pws_ = nullptr;
    double* us_ = nullptr;
    double* alphas_ = nullptr;
    double* pcs_ = nullptr;
    int* signs_ = nullptr;

    // 便宜上のカメラモデル
    static constexpr float fx_ = 1.0, fy_ = 1.0, cx_ = 0.0, cy_ = 0.0;

    double cws[4][3], ccs[4][3];

    unsigned int num_correspondences_ = 0;
    unsigned int max_num_correspondences_ = 0;
};

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_PNP_SOLVER_H
