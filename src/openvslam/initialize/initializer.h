#ifndef OPENVSLAM_INITIALIZE_INITIALIZER_H
#define OPENVSLAM_INITIALIZE_INITIALIZER_H

#include "openvslam/data/frame.h"

#include <memory>

namespace openvslam {

class config;

namespace data {
class frame;
class map_database;
class bow_database;
} // namespace data

namespace initialize {

//! initialization status
enum class status_t {
    NotReady,
    Initializing,
    Wrong,
    Succeeded
};

class base;

class initializer {
public:
    /**
     * Constructor
     * @param cfg
     * @param map_db
     * @param bow_db
     */
    initializer(const std::shared_ptr<config>& cfg, data::map_database* map_db, data::bow_database* bow_db);

    /**
     * Destructor
     */
    ~initializer();

    /**
     * Reset initializer
     */
    void reset();

    /**
     * Get initialization status
     * @return
     */
    status_t get_status() const;

    /**
     * Get keypoints of the initial frame
     * @return
     */
    std::vector<cv::KeyPoint> get_initial_keypoints() const;

    /**
     * Get initial matches between the initial and current frames
     * @return
     */
    std::vector<int> get_initial_matches() const;

    /**
     * Initialize with the current frame
     * @param curr_frm
     * @return
     */
    bool initialize(data::frame& curr_frm);

private:
    //! config
    std::shared_ptr<config> cfg_ = nullptr;
    //! map database
    data::map_database* map_db_ = nullptr;
    //! BoW database
    data::bow_database* bow_db_ = nullptr;
    //! initializer status
    status_t status_ = status_t::NotReady;

    //-----------------------------------------
    // for monocular camera model

    /**
     * Create initializer for monocular
     * @param curr_frm
     */
    void create_initializer(data::frame& curr_frm);

    /**
     * Try to initialize a map with monocular camera setup
     * @param curr_frm
     * @return initial map should be created or not
     */
    bool try_initialize_for_monocular(data::frame& curr_frm);

    /**
     * Create an initial map with monocular camera setup
     * @param curr_frm
     * @return initial map is successfully created or not
     */
    bool create_map_for_monocular(data::frame& curr_frm);

    /**
     * Scaling up or down a initial map
     * @param scale
     */
    void scale_map(data::keyframe* init_keyfrm, data::keyframe* curr_keyfrm, const double scale);

    //! initializer for monocular
    initialize::base* initializer_ = nullptr;
    //! initial frame
    data::frame init_frm_;
    //! coordinates of previously matched points to perform area-based matching
    std::vector<cv::Point2f> prev_matched_coords_;
    //! initial matching indices (index: idx of initial frame, value: idx of current frame)
    std::vector<int> init_matches_;

    //-----------------------------------------
    // for stereo or RGBD camera model

    /**
     * Try to initialize a map with stereo or RGBD camera setup
     * @param curr_frm
     * @return initial map should be created or not
     */
    bool try_initialize_for_stereo(data::frame& curr_frm);

    /**
     * Create an initial map with stereo or RGBD camera setup
     * @param curr_frm
     * @return initial map is successfully created or not
     */
    bool create_map_for_stereo(data::frame& curr_frm);
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_INITIALIZER_H
