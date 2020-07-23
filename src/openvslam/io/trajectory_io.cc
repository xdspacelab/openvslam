#include "openvslam/data/keyframe.h"
#include "openvslam/data/map_database.h"
#include "openvslam/io/trajectory_io.h"

#include <iostream>
#include <iomanip>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace openvslam {
namespace io {

trajectory_io::trajectory_io(data::map_database* map_db)
    : map_db_(map_db) {}

void trajectory_io::save_frame_trajectory(const std::string& path, const std::string& format) const {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. acquire the frame stats

    assert(map_db_);
    const auto frm_stats = map_db_->get_frame_statistics();

    // 2. save the frames

    const auto num_valid_frms = frm_stats.get_num_valid_frames();
    const auto reference_keyframes = frm_stats.get_reference_keyframes();
    const auto rel_cam_poses_from_ref_keyfrms = frm_stats.get_relative_cam_poses();
    const auto timestamps = frm_stats.get_timestamps();
    const auto is_lost_frms = frm_stats.get_lost_frames();

    if (num_valid_frms == 0) {
        spdlog::warn("there are no valid frames, cannot dump frame trajectory");
        return;
    }

    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) {
        spdlog::critical("cannot create a file at {}", path);
        throw std::runtime_error("cannot create a file at " + path);
    }

    spdlog::info("dump frame trajectory in \"{}\" format from frame {} to frame {} ({} frames)",
                 format, reference_keyframes.begin()->first, reference_keyframes.rbegin()->first, num_valid_frms);

    const auto rk_itr_bgn = reference_keyframes.begin();
    const auto rc_itr_bgn = rel_cam_poses_from_ref_keyfrms.begin();
    const auto rk_itr_end = reference_keyframes.end();
    const auto rc_itr_end = rel_cam_poses_from_ref_keyfrms.end();
    auto rk_itr = rk_itr_bgn;
    auto rc_itr = rc_itr_bgn;

    int offset = rk_itr->first;
    unsigned int prev_frm_id = 0;
    for (unsigned int i = 0; i < num_valid_frms; ++i, ++rk_itr, ++rc_itr) {
        // check frame ID
        assert(rk_itr->first == rc_itr->first);
        const auto frm_id = rk_itr->first;

        // check if the frame was lost or not
        if (is_lost_frms.at(frm_id)) {
            spdlog::warn("frame {} was lost", frm_id);
            continue;
        }

        // check if the frame was skipped or not
        if (frm_id != i + offset) {
            spdlog::warn("frame(s) from {} to {} was/were skipped", prev_frm_id + 1, frm_id - 1);
            offset = frm_id - i;
        }

        auto ref_keyfrm = rk_itr->second;
        const Mat44_t cam_pose_rw = ref_keyfrm->get_cam_pose();
        const Mat44_t rel_cam_pose_cr = rc_itr->second;

        const Mat44_t cam_pose_cw = rel_cam_pose_cr * cam_pose_rw;
        const Mat44_t cam_pose_wc = cam_pose_cw.inverse();

        if (format == "KITTI") {
            ofs << std::setprecision(9)
                << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
                << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
                << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << std::endl;
        }
        else if (format == "TUM") {
            const Mat33_t& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
            const Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
            const Quat_t quat_wc = Quat_t(rot_wc);
            ofs << std::setprecision(15)
                << timestamps.at(frm_id) << " "
                << std::setprecision(9)
                << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
        }
        else {
            throw std::runtime_error("Not implemented: trajectory format \"" + format + "\"");
        }

        prev_frm_id = frm_id;
    }

    if (rk_itr != rk_itr_end || rc_itr != rc_itr_end) {
        spdlog::error("the sizes of frame statistics are not matched");
    }

    ofs.close();
}

void trajectory_io::save_keyframe_trajectory(const std::string& path, const std::string& format) const {
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    // 1. acquire keyframes and sort them

    assert(map_db_);
    auto keyfrms = map_db_->get_all_keyframes();
    std::sort(keyfrms.begin(), keyfrms.end(), [&](const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2) {
        return *keyfrm_1 < *keyfrm_2;
    });

    // 2. save the keyframes

    if (keyfrms.empty()) {
        spdlog::warn("there are no valid keyframes, cannot dump keyframe trajectory");
        return;
    }

    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) {
        spdlog::critical("cannot create a file at {}", path);
        throw std::runtime_error("cannot create a file at " + path);
    }

    spdlog::info("dump keyframe trajectory in \"{}\" format from keyframe {} to keyframe {} ({} keyframes)",
                 format, (*keyfrms.begin())->id_, (*keyfrms.rbegin())->id_, keyfrms.size());

    for (const auto keyfrm : keyfrms) {
        const Mat44_t cam_pose_cw = keyfrm->get_cam_pose();
        const Mat44_t cam_pose_wc = cam_pose_cw.inverse();
        const auto timestamp = keyfrm->timestamp_;

        if (format == "KITTI") {
            ofs << std::setprecision(9)
                << cam_pose_wc(0, 0) << " " << cam_pose_wc(0, 1) << " " << cam_pose_wc(0, 2) << " " << cam_pose_wc(0, 3) << " "
                << cam_pose_wc(1, 0) << " " << cam_pose_wc(1, 1) << " " << cam_pose_wc(1, 2) << " " << cam_pose_wc(1, 3) << " "
                << cam_pose_wc(2, 0) << " " << cam_pose_wc(2, 1) << " " << cam_pose_wc(2, 2) << " " << cam_pose_wc(2, 3) << std::endl;
        }
        else if (format == "TUM") {
            const Mat33_t& rot_wc = cam_pose_wc.block<3, 3>(0, 0);
            const Vec3_t& trans_wc = cam_pose_wc.block<3, 1>(0, 3);
            const Quat_t quat_wc = Quat_t(rot_wc);
            ofs << std::setprecision(15)
                << timestamp << " "
                << std::setprecision(9)
                << trans_wc(0) << " " << trans_wc(1) << " " << trans_wc(2) << " "
                << quat_wc.x() << " " << quat_wc.y() << " " << quat_wc.z() << " " << quat_wc.w() << std::endl;
        }
        else {
            throw std::runtime_error("Not implemented: trajectory format \"" + format + "\"");
        }
    }

    ofs.close();
}

} // namespace io
} // namespace openvslam
