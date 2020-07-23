#ifndef OPENVSLAM_PUBLISH_MAP_PUBLISHER_H
#define OPENVSLAM_PUBLISH_MAP_PUBLISHER_H

#include "openvslam/type.h"

#include <mutex>
#include <memory>

namespace openvslam {

class config;

namespace data {
class keyframe;
class landmark;
class map_database;
} // namespace data

namespace publish {

class map_publisher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     * @param cfg
     * @param map_db
     */
    map_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db);

    /**
     * Destructor
     */
    virtual ~map_publisher();

    /**
     * Set current camera pose
     * NOTE: should be accessed from tracker thread
     * @param cam_pose_cw
     */
    void set_current_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Get current camera pose
     * NOTE: should be accessed from viewer thread
     * @return
     */
    Mat44_t get_current_cam_pose();

    /**
     * Get all keyframes
     * @param all_keyfrms
     * @return number of keyframes in map
     */
    unsigned int get_keyframes(std::vector<std::shared_ptr<data::keyframe>>& all_keyfrms);

    /**
     * Get all landmarks and local landmarks
     * @param all_landmarks
     * @param local_landmarks
     * @return number of landmarks in map
     */
    unsigned int get_landmarks(std::vector<std::shared_ptr<data::landmark>>& all_landmarks,
                               std::set<std::shared_ptr<data::landmark>>& local_landmarks);

private:
    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;

    // -------------------------------------------
    //! mutex to access camera pose
    std::mutex mtx_cam_pose_;
    Mat44_t cam_pose_cw_ = Mat44_t::Identity();
};

} // namespace publish
} // namespace openvslam

#endif // OPENVSLAM_PUBLISH_MAP_PUBLISHER_H
