#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/optimize/transform_optimizer.h"
#include "openvslam/optimize/internal/sim3/transform_vertex.h"
#include "openvslam/optimize/internal/sim3/mutual_reproj_edge_wrapper.h"

#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

transform_optimizer::transform_optimizer(const bool fix_scale, const unsigned int num_iter)
    : fix_scale_(fix_scale), num_iter_(num_iter) {}

unsigned int transform_optimizer::optimize(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                           std::vector<data::landmark*>& matched_lms_in_keyfrm_2,
                                           g2o::Sim3& g2o_Sim3_12, const float chi_sq) const {
    const float sqrt_chi_sq = std::sqrt(chi_sq);

    // 1. optimizerを構築

    auto linear_solver = g2o::make_unique<g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    // 1. Sim3 transformのvertexを作成

    auto Sim3_12_vtx = new internal::sim3::transform_vertex();
    Sim3_12_vtx->setId(0);
    Sim3_12_vtx->setEstimate(g2o_Sim3_12);
    Sim3_12_vtx->setFixed(false);
    Sim3_12_vtx->fix_scale_ = fix_scale_;
    Sim3_12_vtx->rot_1w_ = keyfrm_1->get_rotation();
    Sim3_12_vtx->trans_1w_ = keyfrm_1->get_translation();
    Sim3_12_vtx->rot_2w_ = keyfrm_2->get_rotation();
    Sim3_12_vtx->trans_2w_ = keyfrm_2->get_translation();
    optimizer.addVertex(Sim3_12_vtx);

    // 2. landmarkの拘束を追加

    // 以下の2つのedgeを包含するwrapper
    // - keyfrm_2で観測している3次元点をkeyfrm_1に再投影するconstraint edge(カメラモデルはkeyfrm_1のものに従う)
    // - keyfrm_1で観測している3次元点をkeyfrm_2に再投影するconstraint edge(カメラモデルはkeyfrm_2のものに従う)
    using reproj_edge_wrapper = internal::sim3::mutual_reproj_edge_wapper<data::keyframe>;
    std::vector<reproj_edge_wrapper> mutual_edges;
    // 対応数
    const unsigned int num_matches = matched_lms_in_keyfrm_2.size();
    mutual_edges.reserve(num_matches);

    // keyfrm_1で観測している全3次元点
    const auto lms_in_keyfrm_1 = keyfrm_1->get_landmarks();

    // 有効な対応数
    unsigned int num_valid_matches = 0;

    for (unsigned int idx1 = 0; idx1 < num_matches; ++idx1) {
        // matching情報があるもののみを対象とする
        if (!matched_lms_in_keyfrm_2.at(idx1)) {
            continue;
        }

        auto lm_1 = lms_in_keyfrm_1.at(idx1);
        auto lm_2 = matched_lms_in_keyfrm_2.at(idx1);

        // 3次元点が両方存在するもののみを対象とする
        if (!lm_1 || !lm_2) {
            continue;
        }
        if (lm_1->will_be_erased() || lm_2->will_be_erased()) {
            continue;
        }

        const auto idx2 = lm_2->get_index_in_keyframe(keyfrm_2);

        if (idx2 < 0) {
            continue;
        }

        // forward/backward edgesを作成してoptimizerにセット
        reproj_edge_wrapper mutual_edge(keyfrm_1, idx1, lm_1, keyfrm_2, idx2, lm_2, Sim3_12_vtx, sqrt_chi_sq);
        optimizer.addEdge(mutual_edge.edge_12_);
        optimizer.addEdge(mutual_edge.edge_21_);

        ++num_valid_matches;
        mutual_edges.push_back(mutual_edge);
    }

    // 3. 最適化を実行

    optimizer.initializeOptimization();
    optimizer.optimize(5);

    // 4. outlierを外す処理

    unsigned int num_outliers = 0;
    for (unsigned int i = 0; i < num_valid_matches; ++i) {
        auto edge_12 = mutual_edges.at(i).edge_12_;
        auto edge_21 = mutual_edges.at(i).edge_21_;

        // inlierチェック
        if (edge_12->chi2() < chi_sq && edge_21->chi2() < chi_sq) {
            continue;
        }

        // outlierにする処理
        const auto idx1 = mutual_edges.at(i).idx1_;
        matched_lms_in_keyfrm_2.at(idx1) = nullptr;

        mutual_edges.at(i).set_as_outlier();
        ++num_outliers;
    }

    if (num_valid_matches - num_outliers < 10) {
        return 0;
    }

    // 5. もう一度最適化

    optimizer.initializeOptimization();
    optimizer.optimize(num_iter_);

    // 6. inlierを数える

    unsigned int num_inliers = 0;
    for (unsigned int i = 0; i < num_valid_matches; ++i) {
        auto edge_12 = mutual_edges.at(i).edge_12_;
        auto edge_21 = mutual_edges.at(i).edge_21_;

        // outlierチェック
        if (mutual_edges.at(i).is_outlier()) {
            continue;
        }

        // outlierチェック
        if (chi_sq < edge_12->chi2() || chi_sq < edge_21->chi2()) {
            // outlierにする処理
            unsigned int idx1 = mutual_edges.at(i).idx1_;
            matched_lms_in_keyfrm_2.at(idx1) = nullptr;
            continue;
        }

        ++num_inliers;
    }

    // 7. 結果を取り出す

    g2o_Sim3_12 = Sim3_12_vtx->estimate();

    return num_inliers;
}

} // namespace optimize
} // namespace openvslam
