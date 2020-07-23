#ifndef OPENVSLAM_MODULE_LOCAL_MAP_CLEANER_H
#define OPENVSLAM_MODULE_LOCAL_MAP_CLEANER_H

#include <list>

namespace openvslam {

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace module {

class local_map_cleaner {
public:
    /**
     * Constructor
     */
    explicit local_map_cleaner(const bool is_monocular);

    /**
     * Destructor
     */
    ~local_map_cleaner() = default;

    /**
     * Set the origin keyframe ID
     */
    void set_origin_keyframe_id(const unsigned int id) {
        origin_keyfrm_id_ = id;
    }

    /**
     * Add fresh landmark to check their redundancy
     */
    void add_fresh_landmark(std::shared_ptr<data::landmark>& lm) {
        fresh_landmarks_.push_back(lm);
    }

    /**
     * Reset the buffer
     */
    void reset();

    /**
     * Remove redundant landmarks
     */
    unsigned int remove_redundant_landmarks(const unsigned int cur_keyfrm_id);

    /**
     * Remove redundant keyframes
     */
    unsigned int remove_redundant_keyframes(const std::shared_ptr<data::keyframe>& cur_keyfrm) const;

    /**
     * Count the valid and the redundant observations in the specified keyframe
     */
    void count_redundant_observations(const std::shared_ptr<data::keyframe>& keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

private:
    //! origin keyframe ID
    unsigned int origin_keyfrm_id_ = 0;

    //! flag which indicates the tracking camera is monocular or not
    const bool is_monocular_;

    //! fresh landmarks to check their redundancy
    std::list<std::shared_ptr<data::landmark>> fresh_landmarks_;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_LOCAL_MAP_CLEANER_H
