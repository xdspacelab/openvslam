#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/frame_statistics.h"

namespace openvslam {
namespace data {

void frame_statistics::update_frame_statistics(const data::frame& frm, const bool is_lost) {
    if (frm.cam_pose_cw_is_valid_) {
        const Mat44_t rel_cam_pose_from_ref_keyfrm = frm.cam_pose_cw_ * frm.ref_keyfrm_->get_cam_pose_inv();

        frm_ids_of_ref_keyfrms_[frm.ref_keyfrm_].push_back(frm.id_);

        ++num_valid_frms_;
        assert(!ref_keyfrms_.count(frm.id_));
        ref_keyfrms_[frm.id_] = frm.ref_keyfrm_;
        assert(!rel_cam_poses_from_ref_keyfrms_.count(frm.id_));
        rel_cam_poses_from_ref_keyfrms_[frm.id_] = rel_cam_pose_from_ref_keyfrm;
        assert(!timestamps_.count(frm.id_));
        timestamps_[frm.id_] = frm.timestamp_;
    }

    assert(!is_lost_frms_.count(frm.id_));
    is_lost_frms_[frm.id_] = is_lost;
}

void frame_statistics::replace_reference_keyframe(data::keyframe* old_keyfrm, data::keyframe* new_keyfrm) {
    // Delete keyframes and update associations.

    assert(num_valid_frms_ == rel_cam_poses_from_ref_keyfrms_.size());
    assert(num_valid_frms_ == ref_keyfrms_.size());
    assert(num_valid_frms_ == timestamps_.size());
    assert(num_valid_frms_ <= is_lost_frms_.size());

    // Finish if no need to replace keyframes
    if (!frm_ids_of_ref_keyfrms_.count(old_keyfrm)) {
        return;
    }

    // Search frames referencing old_keyfrm which is to be deleted.
    const auto frm_ids = frm_ids_of_ref_keyfrms_.at(old_keyfrm);

    for (const auto frm_id : frm_ids) {
        assert(*ref_keyfrms_.at(frm_id) == *old_keyfrm);

        // Get pose and relative pose of the old keyframe
        const Mat44_t old_ref_cam_pose_cw = old_keyfrm->get_cam_pose();
        const Mat44_t old_rel_cam_pose_cr = rel_cam_poses_from_ref_keyfrms_.at(frm_id);

        // Replace pointer of the keyframe to new_keyfrm
        ref_keyfrms_.at(frm_id) = new_keyfrm;

        // Update relative pose
        const Mat44_t new_ref_cam_pose_cw = new_keyfrm->get_cam_pose();
        const Mat44_t new_rel_cam_pose_cr = old_rel_cam_pose_cr * old_ref_cam_pose_cw * new_ref_cam_pose_cw.inverse();
        rel_cam_poses_from_ref_keyfrms_.at(frm_id) = new_rel_cam_pose_cr;
    }

    // Update frames referencing new_keyfrm
    auto& new_frm_ids = frm_ids_of_ref_keyfrms_[new_keyfrm];
    new_frm_ids.insert(new_frm_ids.end(), frm_ids.begin(), frm_ids.end());
    // Remove frames referencing old_keyfrm
    frm_ids_of_ref_keyfrms_.erase(old_keyfrm);
}

std::unordered_map<data::keyframe*, std::vector<unsigned int>> frame_statistics::get_frame_id_of_reference_keyframes() const {
    return frm_ids_of_ref_keyfrms_;
}

unsigned int frame_statistics::get_num_valid_frames() const {
    return num_valid_frms_;
}

std::map<unsigned int, data::keyframe*> frame_statistics::get_reference_keyframes() const {
    return {ref_keyfrms_.begin(), ref_keyfrms_.end()};
}

eigen_alloc_map<unsigned int, Mat44_t> frame_statistics::get_relative_cam_poses() const {
    return {rel_cam_poses_from_ref_keyfrms_.begin(), rel_cam_poses_from_ref_keyfrms_.end()};
}

std::map<unsigned int, double> frame_statistics::get_timestamps() const {
    return {timestamps_.begin(), timestamps_.end()};
}

std::map<unsigned int, bool> frame_statistics::get_lost_frames() const {
    return {is_lost_frms_.begin(), is_lost_frms_.end()};
}

void frame_statistics::clear() {
    num_valid_frms_ = 0;
    frm_ids_of_ref_keyfrms_.clear();
    ref_keyfrms_.clear();
    rel_cam_poses_from_ref_keyfrms_.clear();
    timestamps_.clear();
    is_lost_frms_.clear();
}

} // namespace data
} // namespace openvslam
