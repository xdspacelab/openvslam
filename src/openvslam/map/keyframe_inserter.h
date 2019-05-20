#ifndef OPENVSLAM_MAP_KEYFRAME_INSERTER_H
#define OPENVSLAM_MAP_KEYFRAME_INSERTER_H

#include "openvslam/camera/base.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"

#include <memory>

namespace openvslam {

namespace data {
class map_database;
} // namespace data

namespace map {

class local_mapper;

class keyframe_inserter {
public:
    keyframe_inserter(const camera::setup_type_t setup_type, const float true_depth_thr,
                      data::map_database* map_db, data::bow_database* bow_db,
                      const unsigned int min_num_frms, const unsigned int max_num_frms);

    virtual ~keyframe_inserter() = default;

    void set_local_mapper(map::local_mapper* local_mapper);

    void reset();

    /**
     * Check the new keyframe is needed or not
     * @param curr_frm
     * @param num_tracked_lms
     * @param ref_keyfrm
     * @return
     */
    bool new_keyframe_is_needed(const data::frame& curr_frm, const unsigned int num_tracked_lms,
                                const data::keyframe& ref_keyfrm) const;

    /**
     * Insert the new keyframe derived from the current frame
     * @param curr_frm
     * @return
     */
    data::keyframe* insert_new_keyframe(data::frame& curr_frm);

private:
    /**
     * Queue the new keyframe to the local mapper
     * @param keyfrm
     * @return
     */
    void queue_keyframe(data::keyframe* keyfrm);

    //! setup type of the tracking camera
    const camera::setup_type_t setup_type_;
    //! depth threshold in metric scale
    const float true_depth_thr_;

    //! map database
    data::map_database* map_db_;
    //! BoW database
    data::bow_database* bow_db_;

    //! local mapper
    map::local_mapper* local_mapper_;

    //! min number of frames to insert keyframe
    const unsigned int min_num_frms_;
    //! max number of frames to insert keyframe
    const unsigned int max_num_frms_;

    //! frame ID of the last keyframe
    unsigned int frm_id_of_last_keyfrm_ = 0;
};

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_KEYFRAME_INSERTER_H
