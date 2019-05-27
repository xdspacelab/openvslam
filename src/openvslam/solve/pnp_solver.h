#ifndef OPENVSLAM_SOLVE_PNP_SOLVER_H
#define OPENVSLAM_SOLVE_PNP_SOLVER_H

#include "openvslam/data/landmark.h"
#include "openvslam/data/frame.h"
#include "openvslam/util/converter.h"

#include <opencv2/core/core.hpp>

namespace openvslam {
namespace solve {

class pnp_solver {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    pnp_solver(const eigen_alloc_vector<Vec3_t>& bearings, const std::vector<cv::KeyPoint>& keypts,
               const std::vector<float>& scale_factors, const std::vector<data::landmark*>& assoc_lms);

    ~pnp_solver();

    void set_ransac_parameters(const float probability = 0.99, const unsigned int min_num_inliers = 10, const unsigned int max_num_iters = 500);

    bool estimate();

    std::vector<unsigned int> get_inlier_indices() const {
        std::vector<unsigned int> inlier_indices;
        inlier_indices.reserve(valid_indices_.size());
        for (unsigned int i = 0; i < valid_indices_.size(); ++i) {
            if (best_is_inlier_.at(i)) {
                inlier_indices.push_back(valid_indices_.at(i));
            }
        }
        return inlier_indices;
    }

    Mat33_t get_best_rot_cw() const {
        return best_rot_cw_;
    }

    Vec3_t get_best_trans_cw() const {
        return best_trans_cw_;
    }

    Mat44_t get_best_cam_pose_cw() const {
        return util::converter::to_eigen_cam_pose(best_rot_cw_, best_trans_cw_);
    }

    static constexpr unsigned int min_num_correspondences_ = 4;

private:
    unsigned int count_inliers(const Mat33_t& rot_cw, const Vec3_t& trans_cw, std::vector<bool>& is_inlier);

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

    unsigned int num_matches_ = 0;
    unsigned int max_num_correspondences_ = 0;

    // 対応する2D-3D点の情報
    //! 有効な対応数
    unsigned int num_valid_correspondences_;

    // 以下のvectorは要素ごとに対応している
    //! 有効な対応の特徴点index
    std::vector<unsigned int> valid_indices_;
    //! bearing vector
    eigen_alloc_vector<Vec3_t> valid_bearings_;
    //! 3次元点
    eigen_alloc_vector<Vec3_t> valid_landmarks_;
    //! 許容する最大誤差
    std::vector<float> max_cos_errors_;

    //! RANSACのベストモデル
    Mat33_t best_rot_cw_;
    Vec3_t best_trans_cw_;
    std::vector<bool> best_is_inlier_;

    //! RANSACのパラメータ
    float probability_;
    unsigned int min_num_inliers_;
    unsigned int max_num_iters_;
};

} // namespace solve
} // namespace openvslam

#endif // OPENVSLAM_SOLVE_PNP_SOLVER_H
