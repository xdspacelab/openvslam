#include "openvslam/mapping_module.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/module/loop_bundle_adjuster.h"
#include "openvslam/optimize/global_bundle_adjuster.h"

#include <thread>

#include <spdlog/spdlog.h>

namespace openvslam {
namespace module {

loop_bundle_adjuster::loop_bundle_adjuster(data::map_database* map_db, const unsigned int num_iter)
    : map_db_(map_db), num_iter_(num_iter) {}

void loop_bundle_adjuster::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
}

void loop_bundle_adjuster::count_loop_BA_execution() {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    ++num_exec_loop_BA_;
}

void loop_bundle_adjuster::abort() {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    abort_loop_BA_ = true;
}

bool loop_bundle_adjuster::is_running() const {
    std::lock_guard<std::mutex> lock(mtx_thread_);
    return loop_BA_is_running_;
}

void loop_bundle_adjuster::optimize(const unsigned int identifier) {
    spdlog::info("start loop bundle adjustment");

    unsigned int num_exec_loop_BA = 0;
    {
        std::lock_guard<std::mutex> lock(mtx_thread_);
        loop_BA_is_running_ = true;
        abort_loop_BA_ = false;
        num_exec_loop_BA = num_exec_loop_BA_;
    }

    const auto global_bundle_adjuster = optimize::global_bundle_adjuster(map_db_, num_iter_, false);
    global_bundle_adjuster.optimize(identifier, &abort_loop_BA_);

    {
        std::lock_guard<std::mutex> lock1(mtx_thread_);

        // if count_loop_BA_execution() was called during the loop BA or the loop BA was aborted,
        // cannot update the map
        if (num_exec_loop_BA != num_exec_loop_BA_ || abort_loop_BA_) {
            spdlog::info("abort loop bundle adjustment");
            loop_BA_is_running_ = false;
            abort_loop_BA_ = false;
            return;
        }

        spdlog::info("finish loop bundle adjustment");
        spdlog::info("updating the map with pose propagation");

        // stop mapping module
        mapper_->request_pause();
        while (!mapper_->is_paused() && !mapper_->is_terminated()) {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));
        }

        std::lock_guard<std::mutex> lock2(data::map_database::mtx_database_);

        // update the camera pose along the spanning tree from the origin
        std::list<std::shared_ptr<data::keyframe>> keyfrms_to_check;
        keyfrms_to_check.push_back(map_db_->origin_keyfrm_);
        while (!keyfrms_to_check.empty()) {
            auto parent = keyfrms_to_check.front();
            const Mat44_t cam_pose_wp = parent->get_cam_pose_inv();

            const auto children = parent->graph_node_->get_spanning_children();
            for (auto child : children) {
                if (child->loop_BA_identifier_ != identifier) {
                    // if `child` is NOT optimized by the loop BA
                    // propagate the pose correction from the spanning parent

                    // parent->child
                    const Mat44_t cam_pose_cp = child->get_cam_pose() * cam_pose_wp;
                    // world->child AFTER correction = parent->child * world->parent AFTER correction
                    child->cam_pose_cw_after_loop_BA_ = cam_pose_cp * parent->cam_pose_cw_after_loop_BA_;
                    // check as `child` has been corrected
                    child->loop_BA_identifier_ = identifier;
                }

                // need updating
                keyfrms_to_check.push_back(child);
            }

            // temporally store the camera pose BEFORE correction (for correction of landmark positions)
            parent->cam_pose_cw_before_BA_ = parent->get_cam_pose();
            // update the camera pose
            parent->set_cam_pose(parent->cam_pose_cw_after_loop_BA_);
            // finish updating
            keyfrms_to_check.pop_front();
        }

        // update the positions of the landmarks
        const auto landmarks = map_db_->get_all_landmarks();
        for (const auto& lm : landmarks) {
            if (lm->will_be_erased()) {
                continue;
            }

            if (lm->loop_BA_identifier_ == identifier) {
                // if `lm` is optimized by the loop BA

                // update with the optimized position
                lm->set_pos_in_world(lm->pos_w_after_global_BA_);
            }
            else {
                // if `lm` is NOT optimized by the loop BA

                // correct the position according to the move of the camera pose of the reference keyframe
                auto ref_keyfrm = lm->get_ref_keyframe();

                assert(ref_keyfrm->loop_BA_identifier_ == identifier);

                // convert the position to the camera-reference using the camera pose BEFORE the correction
                const Mat33_t rot_cw_before_BA = ref_keyfrm->cam_pose_cw_before_BA_.block<3, 3>(0, 0);
                const Vec3_t trans_cw_before_BA = ref_keyfrm->cam_pose_cw_before_BA_.block<3, 1>(0, 3);
                const Vec3_t pos_c = rot_cw_before_BA * lm->get_pos_in_world() + trans_cw_before_BA;

                // convert the position to the world-reference using the camera pose AFTER the correction
                const Mat44_t cam_pose_wc = ref_keyfrm->get_cam_pose_inv();
                const Mat33_t rot_wc = cam_pose_wc.block<3, 3>(0, 0);
                const Vec3_t trans_wc = cam_pose_wc.block<3, 1>(0, 3);
                lm->set_pos_in_world(rot_wc * pos_c + trans_wc);
            }
        }

        mapper_->resume();
        loop_BA_is_running_ = false;

        spdlog::info("updated the map");
    }
}

} // namespace module
} // namespace openvslam
