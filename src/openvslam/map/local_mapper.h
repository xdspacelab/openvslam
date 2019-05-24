#ifndef OPENVSLAM_MAP_LOCAL_MAPPER_H
#define OPENVSLAM_MAP_LOCAL_MAPPER_H

#include "openvslam/camera/base.h"
#include "openvslam/optimize/local_bundle_adjuster.h"

#include <mutex>
#include <atomic>

namespace openvslam {

namespace camera {
class base;
} // namespace camera

namespace data {
class keyframe;
class map_database;
} // namespace data

namespace track {
class tracker;
} // namespace track

namespace map {

class loop_closer;

class local_mapper {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    local_mapper() = delete;

    /**
     * Constructor
     */
    local_mapper(data::map_database* map_db, const bool is_monocular);

    /**
     * Destructor
     */
    ~local_mapper();

    /**
     * Set tracker
     */
    void set_tracker(track::tracker* tracker);

    /**
     * Set loop closer
     */
    void set_loop_closer(loop_closer* loop_closer);

    //-----------------------------------------
    // main process

    /**
     * Run main loop of local mapper
     */
    void run();

    /**
     * Queue a keyframe to process local mapping
     */
    void queue_keyframe(data::keyframe* keyfrm);

    /**
     * Get the number of queued keyframes
     */
    unsigned int get_num_queued_keyframes() const;

    /**
     * Get keyframe acceptability
     */
    bool get_keyframe_acceptability() const;

    /**
     * Set keyframe acceptability
     */
    void set_keyframe_acceptability(const bool acceptability);

    //-----------------------------------------
    // management for reset process

    /**
     * Request to reset the local mapper
     * (NOTE: this function waits for reset)
     */
    void request_reset();

    //-----------------------------------------
    // management for pause process

    /**
     * Request to pause the local mapper
     * (NOTE: this function does not wait for reset)
     */
    void request_pause();

    /**
     * Check if the local mapper is requested to be paused or not
     */
    bool pause_is_requested() const;

    /**
     * Check if the local mapper is paused or not
     */
    bool is_paused() const;

    /**
     * Set the flag to force to run the local mapper
     */
    bool set_force_to_run(const bool force_to_run);

    /**
     * Resume local mapper
     */
    void resume();

    //-----------------------------------------
    // management for terminate process

    /**
     * Request to terminate the local mapper
     * (NOTE: this function does not wait for terminate)
     */
    void request_terminate();

    /**
     * Check if the local mapper is terminated or not
     */
    bool is_terminated() const;

    //-----------------------------------------
    // management for local BA

    /**
     * Abort the local BA externally
     * (NOTE: this function does not wait for abort)
     */
    void abort_local_BA();

private:
    //-----------------------------------------
    // main process

    /**
     * Create and extend the map with the new keyframe
     */
    void mapping_with_new_keyframe();

    /**
     * Store the new keyframe to the map database
     */
    void store_new_keyframe();

    /**
     * Create new landmarks using neighbor keyframes
     */
    void create_new_landmarks();

    /**
     * Triangulate landmarks between the keyframes 1 and 2
     */
    void triangulate_with_two_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                        const std::vector<std::pair<unsigned int, unsigned int>>& matches);

    /**
     * Remove redundant landmarks
     */
    void remove_redundant_landmarks();

    /**
     * Remove redundant keyframes
     */
    void remove_redundant_keyframes();

    /**
     * Count the valid and the redundant observations in the specified keyframe
     */
    void count_redundant_landmarks(data::keyframe* keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

    /**
     * Update the new keyframe
     */
    void update_new_keyframe();

    /**
     * Get the first and second order covisibilities of current keyframe
     */
    std::unordered_set<data::keyframe*> get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                        const unsigned int second_order_thr);

    /**
     * Fuse duplicated landmarks between current keyframe and covisibility keyframes
     */
    void fuse_landmark_duplication(const std::unordered_set<data::keyframe*>& fuse_tgt_keyfrms);

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    /**
     * Check and execute reset
     */
    bool check_reset_request() const;

    /**
     * Reset the variables
     */
    void reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    /**
     * Check and execute pause
     */
    bool check_pause_request() const;

    /**
     * Pause the local mapper
     */
    void pause();

    //! flag which indicates termination is requested or not
    bool pause_is_requested_ = false;
    //! flag which indicates whether the main loop is paused or not
    bool is_paused_ = false;
    //! flag to force the local mapper to be run
    bool force_to_run_ = false;

    //-----------------------------------------
    // management for terminate process

    //! mutex for access to terminate procedure
    mutable std::mutex mtx_terminate_;

    /**
     * Check if termination is requested or not
     */
    bool check_terminate_request() const;

    /**
     * Raise the flag which indicates the main loop has been already terminated
     */
    void terminate();

    //! flag which indicates termination is requested or not
    bool terminate_is_requested_ = false;
    //! flag which indicates whether the main loop is terminated or not
    bool is_terminated_ = true;

    //-----------------------------------------
    // modules

    //! tracker
    track::tracker* tracker_ = nullptr;
    //! loop closer
    loop_closer* loop_closer_ = nullptr;

    //-----------------------------------------
    // database

    //! map database
    data::map_database* map_db_ = nullptr;

    //-----------------------------------------
    // keyframe queue

    //! mutex for access to keyframe queue
    mutable std::mutex mtx_keyfrm_queue_;

    /**
     * Check if keyframe is queued
     */
    bool keyframe_is_queued() const;

    //! queue for keyframes
    std::list<data::keyframe*> keyfrms_queue_;

    //-----------------------------------------
    // optimizer

    //! local bundle adjuster
    const optimize::local_bundle_adjuster local_bundle_adjuster_;

    //! bridge flag to abort local BA
    bool abort_BA_is_requested_ = false;

    //-----------------------------------------
    // others

    //! flag which indicates the tracking camera is monocular or not
    const bool is_monocular_;

    //! flag for keyframe acceptability
    std::atomic<bool> keyframe_acceptability_{true};

    //! current keyframe which is used in the current local mapping
    data::keyframe* cur_keyfrm_ = nullptr;

    //! fresh landmarks to check their redundancy
    std::list<data::landmark*> fresh_landmarks_;
};

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_LOCAL_MAPPER_H
