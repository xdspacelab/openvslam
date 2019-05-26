#ifndef OPENVSLAM_MAP_LOOP_CLOSER_H
#define OPENVSLAM_MAP_LOOP_CLOSER_H

#include "openvslam/type.h"
#include "openvslam/data/bow_vocabulary.h"
#include "openvslam/map/type.h"
#include "openvslam/map/loop_detector.h"
#include "openvslam/map/loop_bundle_adjuster.h"
#include "openvslam/optimize/graph_optimizer.h"

#include <list>
#include <mutex>
#include <thread>

namespace openvslam {

namespace data {
class keyframe;
class bow_database;
class map_database;
} // namespace data

namespace track {
class tracker;
} // namespace track

namespace map {

class local_mapper;

class loop_closer {
public:
    loop_closer() = delete;

    /**
     * Constructor
     */
    loop_closer(data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale);

    /**
     * Destructor
     */
    ~loop_closer();

    /**
     * Set tracker
     */
    void set_tracker(track::tracker* tracker);

    /**
     * Set local mapper
     */
    void set_local_mapper(map::local_mapper* local_mapper);

    //-----------------------------------------
    // interfaces to ON/OFF loop detector

    /**
     * Enable the loop detector
     */
    void enable_loop_detector();

    /**
     * Disable the loop detector
     */
    void disable_loop_detector();

    /**
     * The loop detector is enabled or not
     */
    bool loop_detector_is_enabled() const;

    //-----------------------------------------
    // main process

    /**
     * Run main loop of loop closer
     */
    void run();

    /**
     * Queue a keyframe to the BoW database
     */
    void queue_keyframe(data::keyframe* keyfrm);

    //-----------------------------------------
    // management for reset process

    /**
     * Request to reset the loop closer
     * (NOTE: this function waits for reset)
     */
    void request_reset();

    //-----------------------------------------
    // management for pause process

    /**
     * Request to pause loop closer
     * (NOTE: this function does not wait for pause)
     */
    void request_pause();

    /**
     * Check if the loop closer is requested to be paused or not
     */
    bool pause_is_requested() const;

    /**
     * Check if the loop closer is paused or not
     */
    bool is_paused() const;

    /**
     * Resume loop closer
     */
    void resume();

    //-----------------------------------------
    // management for terminate process

    /**
     * Request to terminate the loop closer
     * (NOTE: this function does not wait for terminate)
     */
    void request_terminate();

    /**
     * Check if the loop closer is terminated or not
     */
    bool is_terminated() const;

    //-----------------------------------------
    // management for loop BA

    /**
     * Check if loop BA is running or not
     */
    bool loop_BA_is_running() const;

    /**
     * Abort the loop BA externally
     * (NOTE: this function does not wait for abort)
     */
    void abort_loop_BA();

private:
    //-----------------------------------------
    // main process

    /**
     * Perform loop closing
     */
    void correct_loop();

    /**
     * Compute Sim3s (world to covisibility) which are prior to loop correction
     */
    keyframe_Sim3_pairs_t get_Sim3s_before_loop_correction(const std::vector<data::keyframe*>& neighbors) const;

    /**
     * Compute Sim3s (world to covisibility) which are corrected using the estimated Sim3 of the current keyframe
     */
    keyframe_Sim3_pairs_t get_Sim3s_after_loop_correction(const Mat44_t& cam_pose_wc_before_correction, const g2o::Sim3& g2o_Sim3_cw_after_correction,
                                                          const std::vector<data::keyframe*>& neighbors) const;

    /**
     * Correct the positions of the landmarks which are seen in covisibilities
     */
    void correct_covisibility_landmarks(const keyframe_Sim3_pairs_t& Sim3s_nw_before_correction,
                                        const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    /**
     * Correct the camera poses of the covisibilities
     */
    void correct_covisibility_keyframes(const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    /**
     * Detect and replace duplicated landmarks
     */
    void replace_duplicated_landmarks(const std::vector<data::landmark*>& curr_match_lms_observed_in_cand,
                                      const keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    /**
     * Extract the new connections which will be created AFTER loop correction
     */
    std::map<data::keyframe*, std::set<data::keyframe*>> extract_new_connections(const std::vector<data::keyframe*>& covisibilities) const;

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    /**
     * Check and execute reset
     */
    bool reset_is_requested() const;

    /**
     * Reset the global optimization module
     */
    void reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    /**
     * Pause the global optimizer
     */
    void pause();

    //! flag which indicates termination is requested or not
    bool pause_is_requested_ = false;
    //! flag which indicates whether the main loop is paused or not
    bool is_paused_ = false;

    //-----------------------------------------
    // management for terminate process

    //! mutex for access to terminate procedure
    mutable std::mutex mtx_terminate_;

    /**
     * Check if termination is requested or not
     */
    bool terminate_is_requested() const;

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
    //! local mapper
    local_mapper* local_mapper_ = nullptr;

    //! loop detector
    loop_detector loop_detector_;
    //! loop bundle adjuster
    loop_bundle_adjuster loop_bundle_adjuster_;

    //-----------------------------------------
    // database

    //! map database
    data::map_database* map_db_;

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

    data::keyframe* cur_keyfrm_ = nullptr;

    //-----------------------------------------
    // optimizer

    //! graph optimizer
    const optimize::graph_optimizer graph_optimizer_;

    //-----------------------------------------
    // variables for loop BA

    //! thread for running loop BA
    std::unique_ptr<std::thread> thread_for_loop_BA_ = nullptr;
};

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_LOOP_CLOSER_H
