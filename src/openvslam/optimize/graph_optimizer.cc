#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/optimize/graph_optimizer.h"
#include "openvslam/optimize/internal/sim3/shot_vertex.h"
#include "openvslam/optimize/internal/sim3/graph_opt_edge.h"
#include "openvslam/util/converter.h"

#include <Eigen/StdVector>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/optimization_algorithm_levenberg.h>

namespace openvslam {
namespace optimize {

graph_optimizer::graph_optimizer(data::map_database* map_db, const bool fix_scale)
    : map_db_(map_db), fix_scale_(fix_scale) {}

void graph_optimizer::optimize(data::keyframe* loop_keyfrm, data::keyframe* curr_keyfrm,
                               const module::keyframe_Sim3_pairs_t& non_corrected_Sim3s,
                               const module::keyframe_Sim3_pairs_t& pre_corrected_Sim3s,
                               const std::map<data::keyframe*, std::set<data::keyframe*>>& loop_connections) const {
    // 1. optimizerを構築

    auto linear_solver = g2o::make_unique<g2o::LinearSolverCSparse<g2o::BlockSolver_7_3::PoseMatrixType>>();
    auto block_solver = g2o::make_unique<g2o::BlockSolver_7_3>(std::move(linear_solver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(algorithm);

    // 2. vertexを追加

    const auto all_keyfrms = map_db_->get_all_keyframes();
    const auto all_lms = map_db_->get_all_landmarks();

    const unsigned int max_keyfrm_id = map_db_->get_max_keyframe_id();

    // 全keyframeの姿勢（修正前)をSim3に変換して保存しておく
    std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> Sim3s_cw(max_keyfrm_id + 1);
    // 追加したvertexを保存しておく
    std::vector<internal::sim3::shot_vertex*> vertices(max_keyfrm_id + 1);

    constexpr int min_weight = 100;

    for (auto keyfrm : all_keyfrms) {
        if (keyfrm->will_be_erased()) {
            continue;
        }
        auto keyfrm_vtx = new internal::sim3::shot_vertex();

        const auto id = keyfrm->id_;

        // 最適化前に姿勢が修正されているかをチェック
        const auto iter = pre_corrected_Sim3s.find(keyfrm);
        if (iter != pre_corrected_Sim3s.end()) {
            // 最適化前に姿勢を修正済みの場合はその姿勢を取り出してvertexにセットする
            Sim3s_cw.at(id) = iter->second;
            keyfrm_vtx->setEstimate(iter->second);
        }
        else {
            // 姿勢が修正されていない場合はkeyframeの姿勢をSim3に変換してセットする
            const Mat33_t rot_cw = keyfrm->get_rotation();
            const Vec3_t trans_cw = keyfrm->get_translation();
            const g2o::Sim3 Sim3_cw(rot_cw, trans_cw, 1.0);

            Sim3s_cw.at(id) = Sim3_cw;
            keyfrm_vtx->setEstimate(Sim3_cw);
        }

        // ループの起点になった点はfixしておく
        if (*keyfrm == *loop_keyfrm) {
            keyfrm_vtx->setFixed(true);
        }

        // vertexをoptimizerにセット
        keyfrm_vtx->setId(id);
        keyfrm_vtx->fix_scale_ = fix_scale_;

        optimizer.addVertex(keyfrm_vtx);
        vertices.at(id) = keyfrm_vtx;
    }

    // 3. edgeを追加

    // どのkeyframe間にedgeが追加されたかを保存しておく
    std::set<std::pair<unsigned int, unsigned int>> inserted_edge_pairs;

    // constraint edgeを追加する関数
    const auto insert_edge =
        [&optimizer, &vertices, &inserted_edge_pairs](unsigned int id1, unsigned int id2, const g2o::Sim3& Sim3_21) {
            auto edge = new internal::sim3::graph_opt_edge();
            edge->setVertex(0, vertices.at(id1));
            edge->setVertex(1, vertices.at(id2));
            edge->setMeasurement(Sim3_21);

            edge->information() = MatRC_t<7, 7>::Identity();

            optimizer.addEdge(edge);
            inserted_edge_pairs.insert(std::make_pair(std::min(id1, id2), std::max(id1, id2)));
        };

    // threshold weight以上のloop edgeを追加する
    for (const auto& loop_connection : loop_connections) {
        auto keyfrm = loop_connection.first;
        const auto& connected_keyfrms = loop_connection.second;

        const auto id1 = keyfrm->id_;
        const g2o::Sim3& Sim3_1w = Sim3s_cw.at(id1);
        const g2o::Sim3 Sim3_w1 = Sim3_1w.inverse();

        for (auto connected_keyfrm : connected_keyfrms) {
            const auto id2 = connected_keyfrm->id_;

            // current vs loop以外のedgeについては，weight thresholdを超えているもののみ
            // を追加する
            if ((id1 != curr_keyfrm->id_ || id2 != loop_keyfrm->id_)
                && keyfrm->graph_node_->get_weight(connected_keyfrm) < min_weight) {
                continue;
            }

            // 相対姿勢を計算
            const g2o::Sim3& Sim3_2w = Sim3s_cw.at(id2);
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // constraint edgeを追加
            insert_edge(id1, id2, Sim3_21);
        }
    }

    // loop connection以外のedgeを追加する
    for (auto keyfrm : all_keyfrms) {
        // 片方のkeyframeの姿勢を取り出す
        const auto id1 = keyfrm->id_;

        // covisibilitiesに含まれているかをチェックし，必ず修正前の姿勢を使うようにする
        // (正しく相対姿勢を計算するには両者が修正前である必要があるため)
        const auto iter1 = non_corrected_Sim3s.find(keyfrm);
        const g2o::Sim3 Sim3_w1 = ((iter1 != non_corrected_Sim3s.end()) ? iter1->second : Sim3s_cw.at(id1)).inverse();

        auto parent_node = keyfrm->graph_node_->get_spanning_parent();
        if (parent_node) {
            const auto id2 = parent_node->id_;

            // 重複を防ぐ
            if (id1 <= id2) {
                continue;
            }

            // covisibilitiesに含まれているかをチェックし，必ず修正前の姿勢を使うようにする
            // (正しく相対姿勢を計算するには両者が修正前である必要があるため)
            const auto iter2 = non_corrected_Sim3s.find(parent_node);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // 相対姿勢を計算
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // constraint edgeを追加
            insert_edge(id1, id2, Sim3_21);
        }

        // loop edgeはweightにかかわらず追加する
        const auto loop_edges = keyfrm->graph_node_->get_loop_edges();
        for (auto connected_keyfrm : loop_edges) {
            const auto id2 = connected_keyfrm->id_;

            // 重複を防ぐ
            if (id1 <= id2) {
                continue;
            }

            // covisibilitiesに含まれているかをチェックし，必ず修正前の姿勢を使うようにする
            // (正しく相対姿勢を計算するには両者が修正前である必要があるため)
            const auto iter2 = non_corrected_Sim3s.find(connected_keyfrm);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // 相対姿勢を計算
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // constraint edgeを追加
            insert_edge(id1, id2, Sim3_21);
        }

        // threshold weight以上のcovisibilitiesを追加する
        const auto connected_keyfrms = keyfrm->graph_node_->get_covisibilities_over_weight(min_weight);
        for (auto connected_keyfrm : connected_keyfrms) {
            // null check
            if (!connected_keyfrm || !parent_node) {
                continue;
            }
            // parent-childのedgeはすでに追加しているので除外
            if (*connected_keyfrm == *parent_node
                || keyfrm->graph_node_->has_spanning_child(connected_keyfrm)) {
                continue;
            }
            // loop対応している場合はすでに追加しているので除外
            if (static_cast<bool>(loop_edges.count(connected_keyfrm))) {
                continue;
            }

            if (connected_keyfrm->will_be_erased()) {
                continue;
            }

            const auto id2 = connected_keyfrm->id_;

            // 重複を防ぐ
            if (id1 <= id2) {
                continue;
            }
            if (static_cast<bool>(inserted_edge_pairs.count(std::make_pair(std::min(id1, id2), std::max(id1, id2))))) {
                continue;
            }

            // covisibilitiesに含まれているかをチェックし，必ず修正前の姿勢を使うようにする
            // (正しく相対姿勢を計算するには両者が修正前である必要があるため)
            const auto iter2 = non_corrected_Sim3s.find(connected_keyfrm);
            const g2o::Sim3& Sim3_2w = (iter2 != non_corrected_Sim3s.end()) ? iter2->second : Sim3s_cw.at(id2);

            // 相対姿勢を計算
            const g2o::Sim3 Sim3_21 = Sim3_2w * Sim3_w1;

            // constraint edgeを追加
            insert_edge(id1, id2, Sim3_21);
        }
    }

    // 4. pose graph optimizationを走らせる

    optimizer.initializeOptimization();
    optimizer.optimize(50);

    // 5. 姿勢を更新

    {
        std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

        // 点群の位置修正のために，全keyframeの姿勢(修正後)を保存しておく
        std::vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> corrected_Sim3s_wc(max_keyfrm_id + 1);

        for (auto keyfrm : all_keyfrms) {
            const auto id = keyfrm->id_;

            auto keyfrm_vtx = static_cast<internal::sim3::shot_vertex*>(optimizer.vertex(id));

            const g2o::Sim3& corrected_Sim3_cw = keyfrm_vtx->estimate();
            const float s = corrected_Sim3_cw.scale();
            const Mat33_t rot_cw = corrected_Sim3_cw.rotation().toRotationMatrix();
            const Vec3_t trans_cw = corrected_Sim3_cw.translation() / s;

            const Mat44_t cam_pose_cw = util::converter::to_eigen_cam_pose(rot_cw, trans_cw);
            keyfrm->set_cam_pose(cam_pose_cw);

            corrected_Sim3s_wc.at(id) = corrected_Sim3_cw.inverse();
        }

        // 点群の位置を修正
        for (auto lm : all_lms) {
            if (lm->will_be_erased()) {
                continue;
            }

            const auto id = (lm->loop_fusion_identifier_ == curr_keyfrm->id_)
                                ? lm->ref_keyfrm_id_in_loop_fusion_
                                : lm->get_ref_keyframe()->id_;

            const g2o::Sim3& Sim3_cw = Sim3s_cw.at(id);
            const g2o::Sim3& corrected_Sim3_wc = corrected_Sim3s_wc.at(id);

            const Vec3_t pos_w = lm->get_pos_in_world();
            const Vec3_t corrected_pos_w = corrected_Sim3_wc.map(Sim3_cw.map(pos_w));

            lm->set_pos_in_world(corrected_pos_w);
            lm->update_normal_and_depth();
        }
    }
}

} // namespace optimize
} // namespace openvslam
