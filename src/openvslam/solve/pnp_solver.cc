#include "openvslam/solve/pnp_solver.h"
#include "openvslam/util/fancy_index.h"
#include "openvslam/util/random_array.h"
#include "openvslam/util/trigonometric.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace solve {

pnp_solver::pnp_solver(const eigen_alloc_vector<Vec3_t>& valid_bearings, const std::vector<cv::KeyPoint>& valid_keypts,
                       const eigen_alloc_vector<Vec3_t>& valid_landmarks, const std::vector<float>& scale_factors,
                       const unsigned int min_num_inliers)
    : num_matches_(valid_bearings.size()), valid_bearings_(valid_bearings),
      valid_landmarks_(valid_landmarks), min_num_inliers_(min_num_inliers) {
    spdlog::debug("CONSTRUCT: solve::pnp_solver");

    max_cos_errors_.clear();
    max_cos_errors_.resize(num_matches_);

    constexpr double max_rad_error = 1.0 * M_PI / 180.0;
    for (unsigned int i = 0; i < num_matches_; ++i) {
        const auto max_rad_error_with_scale = scale_factors.at(valid_keypts.at(i).octave) * max_rad_error;
        max_cos_errors_.at(i) = util::cos(max_rad_error_with_scale);
    }

    assert(num_matches_ == valid_bearings_.size());
    assert(num_matches_ == valid_keypts.size());
    assert(num_matches_ == valid_landmarks_.size());
    assert(num_matches_ == max_cos_errors_.size());
}

pnp_solver::~pnp_solver() {
    delete[] pws_;
    delete[] us_;
    delete[] alphas_;
    delete[] pcs_;
    delete[] signs_;
    spdlog::debug("DESTRUCT: solve::pnp_solver");
}

void pnp_solver::find_via_ransac(const unsigned int max_num_iter, const bool recompute) {
    // 1. Prepare for RANSAC

    // minimum number of samples (= 4)
    static constexpr unsigned int min_set_size = 4;
    if (num_matches_ < min_set_size || num_matches_ < min_num_inliers_) {
        solution_is_valid_ = false;
        return;
    }

    // RANSAC variables
    unsigned int max_num_inliers = 0;
    is_inlier_match = std::vector<bool>(num_matches_, false);

    // shared variables in RANSAC loop
    // rotation from world to camera
    Mat33_t rot_cw_in_sac;
    // translation from world to camera
    Vec3_t trans_cw_in_sac;
    // inlier/outlier flags
    std::vector<bool> is_inlier_match_in_sac;

    // 2. RANSAC loop

    for (unsigned int iter = 0; iter < max_num_iter; ++iter) {
        // 2-1. Create a minimum set
        const auto random_indices = util::create_random_array(min_set_size, 0U, num_matches_ - 1);
        assert(random_indices.size() == min_set_size);
        reset_correspondences();
        set_max_num_correspondences(min_set_size);
        for (const auto i : random_indices) {
            const Vec3_t& bearing = valid_bearings_.at(i);
            const Vec3_t& pos_w = valid_landmarks_.at(i);
            add_correspondence(pos_w, bearing);
        }

        // 2-2. Compute a camera pose
        compute_pose(rot_cw_in_sac, trans_cw_in_sac);

        // 2-3. Check inliers and compute a score
        const auto num_inliers = check_inliers(rot_cw_in_sac, trans_cw_in_sac, is_inlier_match_in_sac);

        // 2-4. Update the best model
        if (max_num_inliers < num_inliers) {
            max_num_inliers = num_inliers;
            best_rot_cw_ = rot_cw_in_sac;
            best_trans_cw_ = trans_cw_in_sac;
            is_inlier_match = is_inlier_match_in_sac;
        }
    }

    if (max_num_inliers > min_num_inliers_) {
        solution_is_valid_ = true;
    }

    if (!recompute || !solution_is_valid_) {
        return;
    }

    // 3. Recompute a camera pose only with the inlier matches

    const auto num_inliers = std::count(is_inlier_match.begin(), is_inlier_match.end(), true);
    reset_correspondences();
    set_max_num_correspondences(num_inliers);
    for (unsigned int i = 0; i < num_matches_; ++i) {
        if (!is_inlier_match.at(i)) {
            continue;
        }
        const Vec3_t& bearing = valid_bearings_.at(i);
        const Vec3_t& pos_w = valid_landmarks_.at(i);
        add_correspondence(pos_w, bearing);
    }

    compute_pose(best_rot_cw_, best_trans_cw_);
}

unsigned int pnp_solver::check_inliers(const Mat33_t& rot_cw, const Vec3_t& trans_cw, std::vector<bool>& is_inlier) {
    unsigned int num_inliers = 0;

    is_inlier.resize(num_matches_);
    for (unsigned int i = 0; i < num_matches_; ++i) {
        const Vec3_t& pos_w = valid_landmarks_.at(i);
        const Vec3_t& bearing = valid_bearings_.at(i);

        const Vec3_t pos_c = rot_cw * pos_w + trans_cw;

        const auto cos = pos_c.dot(bearing) / pos_c.norm();

        if (max_cos_errors_.at(i) < cos) {
            is_inlier.at(i) = true;
            ++num_inliers;
        }
        else {
            is_inlier.at(i) = false;
        }
    }

    return num_inliers;
}

void pnp_solver::reset_correspondences() {
    num_correspondences_ = 0;
}

void pnp_solver::set_max_num_correspondences(const unsigned int max_num_correspondences) {
    delete[] pws_;
    delete[] us_;
    delete[] alphas_;
    delete[] pcs_;
    delete[] signs_;

    max_num_correspondences_ = max_num_correspondences;
    pws_ = new double[3 * max_num_correspondences_];
    us_ = new double[2 * max_num_correspondences_];
    alphas_ = new double[4 * max_num_correspondences_];
    pcs_ = new double[3 * max_num_correspondences_];
    signs_ = new int[max_num_correspondences_];
}

void pnp_solver::add_correspondence(const Vec3_t& pos_w, const Vec3_t& bearing) {
    if (bearing(2) == 0) {
        return;
    }

    pws_[3 * num_correspondences_] = pos_w(0);
    pws_[3 * num_correspondences_ + 1] = pos_w(1);
    pws_[3 * num_correspondences_ + 2] = pos_w(2);

    us_[2 * num_correspondences_] = bearing(0) / bearing(2);
    us_[2 * num_correspondences_ + 1] = bearing(1) / bearing(2);

    if (0.0 < bearing(2)) {
        signs_[num_correspondences_] = 1;
    }
    else {
        signs_[num_correspondences_] = -1;
    }

    ++num_correspondences_;
}

double pnp_solver::compute_pose(Mat33_t& rot_cw, Vec3_t& trans_cw) {
    choose_control_points();
    compute_barycentric_coordinates();

    MatX_t M(2 * num_correspondences_, 12);

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        fill_M(M, 2 * i, alphas_ + 4 * i, us_[2 * i], us_[2 * i + 1]);
    }

    const MatX_t MtM = M.transpose() * M;
    Eigen::JacobiSVD<MatX_t> SVD(MtM, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const MatX_t Ut = SVD.matrixU().transpose();

    MatRC_t<6, 10> L_6x10;
    MatRC_t<6, 1> Rho;

    compute_L_6x10(Ut, L_6x10);
    compute_rho(Rho);

    double Betas[4][4], rep_errors[4];
    double Rs[4][3][3], ts[4][3];

    find_betas_approx_1(L_6x10, Rho, Betas[1]);
    gauss_newton(L_6x10, Rho, Betas[1]);
    rep_errors[1] = compute_R_and_t(Ut, Betas[1], Rs[1], ts[1]);

    find_betas_approx_2(L_6x10, Rho, Betas[2]);
    gauss_newton(L_6x10, Rho, Betas[2]);
    rep_errors[2] = compute_R_and_t(Ut, Betas[2], Rs[2], ts[2]);

    find_betas_approx_3(L_6x10, Rho, Betas[3]);
    gauss_newton(L_6x10, Rho, Betas[3]);
    rep_errors[3] = compute_R_and_t(Ut, Betas[3], Rs[3], ts[3]);

    unsigned int N = 1;
    if (rep_errors[2] < rep_errors[1]) {
        N = 2;
    }
    if (rep_errors[3] < rep_errors[N]) {
        N = 3;
    }

    for (unsigned int r = 0; r < 3; ++r) {
        for (unsigned int c = 0; c < 3; ++c) {
            rot_cw(r, c) = Rs[N][r][c];
        }
    }

    trans_cw(0) = ts[N][0];
    trans_cw(1) = ts[N][1];
    trans_cw(2) = ts[N][2];

    return rep_errors[N];
}

void pnp_solver::choose_control_points() {
    // Take C0 as the reference points centroid:
    cws[0][0] = cws[0][1] = cws[0][2] = 0;
    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            cws[0][j] += pws_[3 * i + j];
        }
    }

    for (unsigned int j = 0; j < 3; ++j) {
        cws[0][j] /= num_correspondences_;
    }

    // Take C1, C2, and C3 from PCA on the reference points:
    MatX_t PW0(num_correspondences_, 3);

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            PW0(i, j) = pws_[3 * i + j] - cws[0][j];
        }
    }

    const MatX_t PW0tPW0 = PW0.transpose() * PW0;
    Eigen::JacobiSVD<MatX_t> SVD(PW0tPW0, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const MatX_t D = SVD.singularValues();
    const MatX_t Ut = SVD.matrixU().transpose();

    for (unsigned int i = 1; i < 4; ++i) {
        const double k = sqrt(D(i - 1, 0) / num_correspondences_);
        for (unsigned int j = 0; j < 3; ++j) {
            cws[i][j] = cws[0][j] + k * Ut((i - 1), j);
        }
    }
}

void pnp_solver::compute_barycentric_coordinates() {
    Mat33_t CC;

    for (int i = 0; i < 3; ++i) {
        for (unsigned int j = 1; j < 4; ++j) {
            CC(i, j - 1) = cws[j][i] - cws[0][i];
        }
    }

    const Mat33_t CC_inv = CC.inverse();

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        double* pi = pws_ + 3 * i;
        double* a = alphas_ + 4 * i;

        for (unsigned int j = 0; j < 3; ++j) {
            a[1 + j] = CC_inv(j, 0) * (pi[0] - cws[0][0])
                       + CC_inv(j, 1) * (pi[1] - cws[0][1])
                       + CC_inv(j, 2) * (pi[2] - cws[0][2]);
        }

        a[0] = 1.0f - a[1] - a[2] - a[3];
    }
}

void pnp_solver::fill_M(MatX_t& M, const int row, const double* as, const double u, const double v) {
    for (unsigned int i = 0; i < 4; ++i) {
        M(row, 3 * i) = as[i] * fx_;
        M(row, 3 * i + 1) = 0.0;
        M(row, 3 * i + 2) = as[i] * (cx_ - u);

        M(row + 1, 3 * i) = 0.0;
        M(row + 1, 3 * i + 1) = as[i] * fy_;
        M(row + 1, 3 * i + 2) = as[i] * (cy_ - v);
    }
}

void pnp_solver::compute_ccs(const double* betas, const MatX_t& ut) {
    for (unsigned int i = 0; i < 4; ++i) {
        ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0;
    }

    for (unsigned int i = 0; i < 4; ++i) {
        for (unsigned int j = 0; j < 4; ++j) {
            for (unsigned int k = 0; k < 3; ++k) {
                ccs[j][k] += betas[i] * ut(11 - i, 3 * j + k);
            }
        }
    }
}

void pnp_solver::compute_pcs() {
    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        double* a = alphas_ + 4 * i;
        double* pc = pcs_ + 3 * i;

        for (unsigned int j = 0; j < 3; ++j) {
            pc[j] = a[0] * ccs[0][j]
                    + a[1] * ccs[1][j]
                    + a[2] * ccs[2][j]
                    + a[3] * ccs[3][j];
        }
    }
}

double pnp_solver::dist2(const double* p1, const double* p2) {
    return (p1[0] - p2[0]) * (p1[0] - p2[0])
           + (p1[1] - p2[1]) * (p1[1] - p2[1])
           + (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double pnp_solver::dot(const double* v1, const double* v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double pnp_solver::reprojection_error(const double R[3][3], const double t[3]) {
    double sum2 = 0.0;

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        double* pw = pws_ + 3 * i;
        double Xc = dot(R[0], pw) + t[0];
        double Yc = dot(R[1], pw) + t[1];
        double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
        double ue = cx_ + fx_ * Xc * inv_Zc;
        double ve = cy_ + fy_ * Yc * inv_Zc;
        double u = us_[2 * i], v = us_[2 * i + 1];

        sum2 += sqrt((u - ue) * (u - ue) + (v - ve) * (v - ve));
    }

    return sum2 / num_correspondences_;
}

void pnp_solver::estimate_R_and_t(double R[3][3], double t[3]) {
    double pc0[3], pw0[3];

    pc0[0] = pc0[1] = pc0[2] = 0.0;
    pw0[0] = pw0[1] = pw0[2] = 0.0;

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        const double* pc = pcs_ + 3 * i;
        const double* pw = pws_ + 3 * i;

        for (unsigned int j = 0; j < 3; ++j) {
            pc0[j] += pc[j];
            pw0[j] += pw[j];
        }
    }
    for (unsigned int j = 0; j < 3; ++j) {
        pc0[j] /= num_correspondences_;
        pw0[j] /= num_correspondences_;
    }

    MatX_t Abt(3, 3);

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            Abt(i, j) = 0.0;
        }
    }

    for (unsigned int i = 0; i < num_correspondences_; ++i) {
        const double* pc = pcs_ + 3 * i;
        const double* pw = pws_ + 3 * i;

        for (unsigned int j = 0; j < 3; ++j) {
            Abt(j, 0) += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
            Abt(j, 1) += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
            Abt(j, 2) += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
        }
    }

    Eigen::JacobiSVD<MatX_t> SVD(Abt, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const MatX_t& Abt_u = SVD.matrixU();
    const MatX_t& Abt_v = SVD.matrixV();

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            R[i][j] = Abt_u.row(i) * Abt_v.row(j).transpose();
        }
    }

    const double det = R[0][0] * R[1][1] * R[2][2]
                       + R[0][1] * R[1][2] * R[2][0]
                       + R[0][2] * R[1][0] * R[2][1]
                       - R[0][2] * R[1][1] * R[2][0]
                       - R[0][1] * R[1][0] * R[2][2]
                       - R[0][0] * R[1][2] * R[2][1];

    //change 1: negative determinant problem is solved by changing Abt_v, not R

    if (det < 0) {
        MatX_t Abt_v_prime = Abt_v;
        Abt_v_prime.col(2) = -Abt_v.col(2);
        for (unsigned int i = 0; i < 3; ++i) {
            for (unsigned int j = 0; j < 3; ++j) {
                R[i][j] = Abt_u.row(i) * Abt_v_prime.row(j).transpose();
            }
        }
    }

    t[0] = pc0[0] - dot(R[0], pw0);
    t[1] = pc0[1] - dot(R[1], pw0);
    t[2] = pc0[2] - dot(R[2], pw0);
}

void pnp_solver::solve_for_sign() {
    //change to this (using original depths)
    if ((pcs_[2] < 0.0 && signs_[0] > 0) || (pcs_[2] > 0.0 && signs_[0] < 0)) {
        for (unsigned int i = 0; i < 4; ++i) {
            for (unsigned int j = 0; j < 3; ++j) {
                ccs[i][j] = -ccs[i][j];
            }
        }

        for (unsigned int i = 0; i < num_correspondences_; ++i) {
            pcs_[3 * i] = -pcs_[3 * i];
            pcs_[3 * i + 1] = -pcs_[3 * i + 1];
            pcs_[3 * i + 2] = -pcs_[3 * i + 2];
        }
    }
}

double pnp_solver::compute_R_and_t(const MatX_t& Ut, const double* betas, double R[3][3], double t[3]) {
    compute_ccs(betas, Ut);
    compute_pcs();

    solve_for_sign();

    estimate_R_and_t(R, t);

    return reprojection_error(R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void pnp_solver::find_betas_approx_1(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas) {
    MatX_t L_6x4(6, 4);

    for (unsigned int i = 0; i < 6; ++i) {
        L_6x4(i, 0) = L_6x10(i, 0);
        L_6x4(i, 1) = L_6x10(i, 1);
        L_6x4(i, 2) = L_6x10(i, 3);
        L_6x4(i, 3) = L_6x10(i, 6);
    }

    Eigen::JacobiSVD<MatX_t> SVD(L_6x4, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const VecX_t Rho_temp = Rho;
    const VecX_t b4 = SVD.solve(Rho_temp);

    if (b4[0] < 0) {
        betas[0] = sqrt(-b4[0]);
        betas[1] = -b4[1] / betas[0];
        betas[2] = -b4[2] / betas[0];
        betas[3] = -b4[3] / betas[0];
    }
    else {
        betas[0] = sqrt(b4[0]);
        betas[1] = b4[1] / betas[0];
        betas[2] = b4[2] / betas[0];
        betas[3] = b4[3] / betas[0];
    }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void pnp_solver::find_betas_approx_2(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas) {
    MatX_t L_6x3(6, 3);

    for (unsigned int i = 0; i < 6; ++i) {
        L_6x3(i, 0) = L_6x10(i, 0);
        L_6x3(i, 1) = L_6x10(i, 1);
        L_6x3(i, 2) = L_6x10(i, 2);
    }

    Eigen::JacobiSVD<MatX_t> SVD(L_6x3, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const VecX_t Rho_temp = Rho;
    const VecX_t b3 = SVD.solve(Rho_temp);

    if (b3[0] < 0) {
        betas[0] = sqrt(-b3[0]);
        betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
    }
    else {
        betas[0] = sqrt(b3[0]);
        betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
    }

    if (b3[1] < 0) {
        betas[0] = -betas[0];
    }

    betas[2] = 0.0;
    betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void pnp_solver::find_betas_approx_3(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double* betas) {
    MatX_t L_6x5(6, 5);

    for (unsigned int i = 0; i < 6; ++i) {
        L_6x5(i, 0) = L_6x10(i, 0);
        L_6x5(i, 1) = L_6x10(i, 1);
        L_6x5(i, 2) = L_6x10(i, 2);
        L_6x5(i, 3) = L_6x10(i, 3);
        L_6x5(i, 4) = L_6x10(i, 4);
    }

    Eigen::JacobiSVD<MatX_t> SVD(L_6x5, Eigen::ComputeFullV | Eigen::ComputeFullU);
    const VecX_t Rho_temp = Rho;
    const VecX_t b5 = SVD.solve(Rho_temp);

    if (b5[0] < 0) {
        betas[0] = sqrt(-b5[0]);
        betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
    }
    else {
        betas[0] = sqrt(b5[0]);
        betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
    }
    if (b5[1] < 0) {
        betas[0] = -betas[0];
    }

    betas[2] = b5[3] / betas[0];
    betas[3] = 0.0;
}

void pnp_solver::compute_L_6x10(const MatX_t& Ut, MatRC_t<6, 10>& L_6x10) {
    double dv[4][6][3];

    for (unsigned int i = 0; i < 4; ++i) {
        unsigned int a = 0, b = 1;
        for (unsigned int j = 0; j < 6; ++j) {
            dv[i][j][0] = Ut(11 - i, 3 * a) - Ut(11 - i, 3 * b);
            dv[i][j][1] = Ut(11 - i, 3 * a + 1) - Ut(11 - i, 3 * b + 1);
            dv[i][j][2] = Ut(11 - i, 3 * a + 2) - Ut(11 - i, 3 * b + 2);

            ++b;
            if (b > 3) {
                ++a;
                b = a + 1;
            }
        }
    }

    for (unsigned int i = 0; i < 6; ++i) {
        L_6x10(i, 0) = dot(dv[0][i], dv[0][i]);
        L_6x10(i, 1) = 2.0f * dot(dv[0][i], dv[1][i]);
        L_6x10(i, 2) = dot(dv[1][i], dv[1][i]);
        L_6x10(i, 3) = 2.0f * dot(dv[0][i], dv[2][i]);
        L_6x10(i, 4) = 2.0f * dot(dv[1][i], dv[2][i]);
        L_6x10(i, 5) = dot(dv[2][i], dv[2][i]);
        L_6x10(i, 6) = 2.0f * dot(dv[0][i], dv[3][i]);
        L_6x10(i, 7) = 2.0f * dot(dv[1][i], dv[3][i]);
        L_6x10(i, 8) = 2.0f * dot(dv[2][i], dv[3][i]);
        L_6x10(i, 9) = dot(dv[3][i], dv[3][i]);
    }
}

void pnp_solver::compute_rho(MatRC_t<6, 1>& Rho) {
    Rho[0] = dist2(cws[0], cws[1]);
    Rho[1] = dist2(cws[0], cws[2]);
    Rho[2] = dist2(cws[0], cws[3]);
    Rho[3] = dist2(cws[1], cws[2]);
    Rho[4] = dist2(cws[1], cws[3]);
    Rho[5] = dist2(cws[2], cws[3]);
}

void pnp_solver::compute_A_and_b_gauss_newton(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho,
                                              double betas[4], MatRC_t<6, 4>& A, MatRC_t<6, 1>& b) {
    for (unsigned int i = 0; i < 6; ++i) {
        A(i, 0) = 2 * L_6x10(i, 0) * betas[0] + L_6x10(i, 1) * betas[1]
                  + L_6x10(i, 3) * betas[2] + L_6x10(i, 6) * betas[3];
        A(i, 1) = L_6x10(i, 1) * betas[0] + 2 * L_6x10(i, 2) * betas[1]
                  + L_6x10(i, 4) * betas[2] + L_6x10(i, 7) * betas[3];
        A(i, 2) = L_6x10(i, 3) * betas[0] + L_6x10(i, 4) * betas[1]
                  + 2 * L_6x10(i, 5) * betas[2] + L_6x10(i, 8) * betas[3];
        A(i, 3) = L_6x10(i, 6) * betas[0] + L_6x10(i, 7) * betas[1]
                  + L_6x10(i, 8) * betas[2] + 2 * L_6x10(i, 9) * betas[3];

        b(i, 0) = Rho[i] - (L_6x10(i, 0) * betas[0] * betas[0] + L_6x10(i, 1) * betas[0] * betas[1] + L_6x10(i, 2) * betas[1] * betas[1] + L_6x10(i, 3) * betas[0] * betas[2] + L_6x10(i, 4) * betas[1] * betas[2] + L_6x10(i, 5) * betas[2] * betas[2] + L_6x10(i, 6) * betas[0] * betas[3] + L_6x10(i, 7) * betas[1] * betas[3] + L_6x10(i, 8) * betas[2] * betas[3] + L_6x10(i, 9) * betas[3] * betas[3]);
    }
}

void pnp_solver::gauss_newton(const MatRC_t<6, 10>& L_6x10, const MatRC_t<6, 1>& Rho, double betas[4]) {
    const int iterations_number = 5;

    MatRC_t<6, 4> A;
    MatRC_t<6, 1> B;
    MatRC_t<4, 1> X;

    for (unsigned int k = 0; k < iterations_number; ++k) {
        compute_A_and_b_gauss_newton(L_6x10, Rho, betas, A, B);
        qr_solve(A, B, X);

        for (unsigned int i = 0; i < 4; ++i) {
            betas[i] += X[i];
        }
    }
}

void pnp_solver::qr_solve(MatRC_t<6, 4>& A_orig, MatRC_t<6, 1>& b, MatRC_t<4, 1>& X) {
    MatRC_t<4, 6> A = A_orig.transpose();

    static int max_nr = 0;
    static double *A1, *A2;

    const int nr = A_orig.rows();
    const int nc = A_orig.cols();

    if (max_nr != 0 && max_nr < nr) {
        delete[] A1;
        delete[] A2;
    }
    if (max_nr < nr) {
        max_nr = nr;
        A1 = new double[nr];
        A2 = new double[nr];
    }

    double* pA = A.data();
    double* ppAkk = pA;
    for (int k = 0; k < nc; ++k) {
        double* ppAik = ppAkk;
        double eta = fabs(*ppAik);
        for (int i = k + 1; i < nr; ++i) {
            const double elt = fabs(*ppAik);
            if (eta < elt) {
                eta = elt;
            }
            ppAik += nc;
        }

        if (eta == 0) {
            A1[k] = A2[k] = 0.0;
            return;
        }
        else {
            double* ppAik = ppAkk;
            double sum = 0.0;
            const double inv_eta = 1.0 / eta;
            for (int i = k; i < nr; ++i) {
                *ppAik *= inv_eta;
                sum += *ppAik * *ppAik;
                ppAik += nc;
            }
            double sigma = sqrt(sum);
            if (*ppAkk < 0) {
                sigma = -sigma;
            }
            *ppAkk += sigma;
            A1[k] = sigma * *ppAkk;
            A2[k] = -eta * sigma;
            for (int j = k + 1; j < nc; ++j) {
                double* ppAik = ppAkk;
                double sum = 0.0;
                for (int i = k; i < nr; i++) {
                    sum += *ppAik * ppAik[j - k];
                    ppAik += nc;
                }
                const double tau = sum / A1[k];
                ppAik = ppAkk;
                for (int i = k; i < nr; ++i) {
                    ppAik[j - k] -= tau * *ppAik;
                    ppAik += nc;
                }
            }
        }
        ppAkk += nc + 1;
    }

    // b <- Qt b
    double* ppAjj = pA;
    double* pb = b.data();
    for (int j = 0; j < nc; ++j) {
        double *ppAij = ppAjj, tau = 0;
        for (int i = j; i < nr; i++) {
            tau += *ppAij * pb[i];
            ppAij += nc;
        }
        tau /= A1[j];
        ppAij = ppAjj;
        for (int i = j; i < nr; ++i) {
            pb[i] -= tau * *ppAij;
            ppAij += nc;
        }
        ppAjj += nc + 1;
    }

    // X = R-1 b
    double* pX = X.data();
    pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
    for (int i = nc - 2; i >= 0; --i) {
        double *ppAij = pA + i * nc + (i + 1), sum = 0;

        for (int j = i + 1; j < nc; ++j) {
            sum += *ppAij * pX[j];
            ppAij++;
        }
        pX[i] = (pb[i] - sum) / A2[i];
    }
}

} // namespace solve
} // namespace openvslam
