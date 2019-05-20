#ifndef OPENVSLAM_DATA_FRAME_STATISTICS_H
#define OPENVSLAM_DATA_FRAME_STATISTICS_H

#include "openvslam/type.h"

#include <vector>
#include <unordered_map>

namespace openvslam {
namespace data {

class frame;
class keyframe;

class frame_statistics {
public:
    /**
     * Constructor
     */
    frame_statistics() = default;

    /**
     * Destructor
     */
    virtual ~frame_statistics() = default;

    /**
     * Update frame statistics
     * @param frm
     * @param is_lost
     */
    void update_frame_statistics(const data::frame& frm, const bool is_lost);

    /**
     * Replace a keyframe which will be erased in frame statistics
     * @param old_keyfrm
     * @param new_keyfrm
     */
    void replace_reference_keyframe(data::keyframe* old_keyfrm, data::keyframe* new_keyfrm);

    /**
     * Get frame IDs of each of the reference keyframes
     * @return
     */
    std::unordered_map<data::keyframe*, std::vector<unsigned int>> get_frame_id_of_reference_keyframes() const;

    /**
     * Get the number of the contained valid frames
     * @return
     */
    unsigned int get_num_valid_frames() const;

    /**
     * Get reference keyframes of each of the frames
     * @return
     */
    std::map<unsigned int, data::keyframe*> get_reference_keyframes() const;

    /**
     * Get relative camera poses from the corresponding reference keyframes
     * @return
     */
    eigen_alloc_map<unsigned int, Mat44_t> get_relative_cam_poses() const;

    /**
     * Get timestamps
     * @return
     */
    std::map<unsigned int, double> get_timestamps() const;

    /**
     * Get lost frame flags
     * @return
     */
    std::map<unsigned int, bool> get_lost_frames() const;

    /**
     * Clear frame statistics
     */
    void clear();

private:
    //! reference keyframeと，そのkeyframeを参照しているframeのID
    std::unordered_map<data::keyframe*, std::vector<unsigned int>> frm_ids_of_ref_keyfrms_;

    //! フレーム数
    unsigned int num_valid_frms_ = 0;
    // 以下のvectorは処理フレーム数だけ要素が存在
    //! 各frameのreference keyframe
    std::unordered_map<unsigned int, data::keyframe*> ref_keyfrms_;
    //! 各frameのreference keyframeに対する相対姿勢
    eigen_alloc_unord_map<unsigned int, Mat44_t> rel_cam_poses_from_ref_keyfrms_;
    //! 各frameのtimestamp
    std::unordered_map<unsigned int, double> timestamps_;
    //! 各frameがロストしたかどうか
    std::unordered_map<unsigned int, bool> is_lost_frms_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_FRAME_STATISTICS_H
