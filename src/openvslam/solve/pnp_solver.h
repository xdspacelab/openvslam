#ifndef OPENVSLAM_SOLVE_PNP_SOLVER_H
#define OPENVSLAM_SOLVE_PNP_SOLVER_H

#include "openvslam/util/converter.h"
#include "openvslam/type.h"

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
    //! (Note: if the number of inliers is less than this, the solution is regarded as invalid)
    const unsigned int min_num_inliers_;

    //! the solution is valid or not
    bool solution_is_valid_ = false;
    //! most reliable rotation
    Mat33_t best_rot_cw_;
    //! most reliable translation
    Vec3_t best_trans_cw_;
    //! inlier matches computed via RANSAC
    std::vector<bool> is_inlier_match;

    //-----------------------------------------
    // quoted from EPnP implementation

    //! Compute camera pose by local bearing vectors and world point positions
    double compute_pose(const eigen_alloc_vector<Vec3_t>& bearing_vectors,
                        const eigen_alloc_vector<Vec3_t>& pos_ws,
                        Mat33_t& rot_cw, Vec3_t& trans_cw);

    //! Reprojecton error on a virtual camera projection surface (intrinsic params are fx_, fy_, cx_ and cy_)
    double reprojection_error(const eigen_alloc_vector<Vec3_t>& pws, const eigen_alloc_vector<Vec3_t>& bearings, const Mat33_t& rot, const Vec3_t& trans);

    //! Choose control points on the world coordinate
    eigen_alloc_vector<Vec3_t> choose_control_points(const eigen_alloc_vector<Vec3_t>& pos_ws);

    //! Compute the barycentric coordinate for each world point using control points
    eigen_alloc_vector<Vec4_t> compute_barycentric_coordinates(const eigen_alloc_vector<Vec3_t>& control_points, const eigen_alloc_vector<Vec3_t>& pos_ws);

    //! Compute M matrix to gain the basis of the local control points
    MatX_t compute_M(const eigen_alloc_vector<Vec3_t>& bearings,
                     const eigen_alloc_vector<Vec4_t>& alphas);

    //! Compute control points on the local coordinate
    eigen_alloc_vector<Vec3_t> compute_ccs(const Vec4_t& betas, const MatX_t& U);

    //! Compute local 3D points by utilize barycentric coordinates(alphas) and local control points(ccs)
    eigen_alloc_vector<Vec3_t> compute_pcs(const eigen_alloc_vector<Vec4_t>& alphas, const eigen_alloc_vector<Vec3_t>& ccs, const bool bearing_z_sign);

    //! Find the coarse value of betas which are coefficients of the basis of the local control points
    Vec4_t find_initial_betas(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho, unsigned int N);

    //! Find the coarse value of betas in the case of N (the number of the non-null space of M) is 2
    Vec4_t find_initial_betas_2(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho);
    //! Find the coarse value of betas in the case of N is 3
    Vec4_t find_initial_betas_3(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho);
    //! Find the coarse value of betas in the case of N is 4
    Vec4_t find_initial_betas_4(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho);

    //! Compute rho vector which is used to solve initial betas
    Vec6_t compute_rho(const eigen_alloc_vector<Vec3_t>& control_points);
    //! Compute L matrix which is used to solve initial betas
    MatRC_t<6, 10> compute_L_6x10(const MatX_t& U);

    //! Compute fine beta using the gauss-newton algorithm
    Vec4_t gauss_newton(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho, const Vec4_t& betas);

    //! Compute A matrix and b vector used for the gauss-newton algorithm
    void compute_A_and_b_for_gauss_newton(const MatRC_t<6, 10>& L_6x10, const Vec6_t& Rho, const Vec4_t& betas, MatRC_t<6, 4>& A, Vec6_t& b);

    //! Estimate R and t by the local 3D points and the world 3D points
    void estimate_R_and_t(const eigen_alloc_vector<Vec3_t>& pws, const eigen_alloc_vector<Vec3_t>& pcs, Mat33_t& rot, Vec3_t& trans);

    // A camera model to utilize bearing vector as a pixel coordinate
    static constexpr float fx_ = 1.0, fy_ = 1.0, cx_ = 0.0, cy_ = 0.0;
};

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_PNP_SOLVER_H
