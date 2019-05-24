#ifndef OPENVSLAM_MAP_LOOP_CLOSER_H
#define OPENVSLAM_MAP_LOOP_CLOSER_H

#include "openvslam/type.h"
#include "openvslam/data/bow_vocabulary.h"
#include "openvslam/map/type.h"
#include "openvslam/map/loop_detector.h"
#include "openvslam/optimize/graph_optimizer.h"
#include "openvslam/optimize/transform_optimizer.h"

#include <list>
#include <thread>
#include <mutex>
#include <atomic>

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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    loop_closer() = delete;

    /**
     * Constructor
     * @param map_db
     * @param bow_db
     * @param bow_vocab
     * @param fix_scale
     */
    loop_closer(data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale);

    /**
     * Destructor
     */
    ~loop_closer();

    /**
     * Set tracker
     * @param tracker
     */
    void set_tracker(track::tracker* tracker);

    /**
     * Set local mapper
     * @param local_mapper
     */
    void set_local_mapper(map::local_mapper* local_mapper);

    //-----------------------------------------
    // interfaces to ON/OFF loop detector

    /**
     * Enable or disable the loop detector
     * @param loop_detector_is_enabled
     */
    void set_loop_detector_status(const bool loop_detector_is_enabled);

    /**
     * Check if the loop detector is enabled or not
     * @return
     */
    bool get_loop_detector_status() const;

    //-----------------------------------------
    // main process

    /**
     * Run main loop of loop closer
     */
    void run();

    /**
     * Queue a keyframe to the BoW database
     * @param keyfrm a keyframe to queue
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
     * @return
     */
    bool pause_is_requested() const;

    /**
     * Check if the loop closer is paused or not
     * @return
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
     * @return whether the loop closer is terminated or not
     */
    bool is_terminated() const;

    //-----------------------------------------
    // management for loop BA

    /**
     * Check if loop BA is running or not
     * @return whether loop BA is running or not
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
     * @param covisibilities
     * @return
     */
    keyframe_Sim3_pairs_t get_non_corrected_Sim3s(const std::vector<data::keyframe*>& covisibilities) const;

    /**
     * Compute Sim3s (world to covisibility) which are corrected using the estimated Sim3 of the current keyframe
     * @param cam_pose_curr_to_world non-corrected camera pose of the current keyframe
     * @param g2o_Sim3_world_to_curr estimated Sim3 of the current keyframe
     * @param covisibilities
     * @return
     */
    keyframe_Sim3_pairs_t get_pre_corrected_Sim3s(const Mat44_t& cam_pose_curr_to_world, const g2o::Sim3& g2o_Sim3_world_to_curr,
                                                  const std::vector<data::keyframe*>& covisibilities) const;

    /**
     * Correct the positions of the landmarks which are seen in covisibilities
     * @param non_corrected_Sim3s_iw
     * @param pre_corrected_Sim3s_iw
     */
    void correct_covisibility_landmarks(const keyframe_Sim3_pairs_t& non_corrected_Sim3s_iw,
                                        const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const;

    /**
     * Correct the camera poses of the covisibilities
     * @param pre_corrected_Sim3s_iw
     */
    void correct_covisibility_keyframes(const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const;

    /**
     * Detect and replace duplicated landmarks
     * @param pre_corrected_Sim3s_iw
     */
    void replace_duplicated_landmarks(const std::vector<data::landmark*>& curr_assoc_lms_in_cand,
                                      const keyframe_Sim3_pairs_t& pre_corrected_Sim3s_iw) const;

    /**
     * Extract the new connections which will be created AFTER loop correction
     * @param covisibilities
     * @return
     */
    std::map<data::keyframe*, std::set<data::keyframe*>> extract_new_connections(const std::vector<data::keyframe*>& covisibilities) const;

    //-----------------------------------------
    // interfaces to ON/OFF loop detector

    std::atomic<bool> loop_detector_is_enabled_{true};

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    /**
     * Check and execute reset
     */
    void check_and_execute_reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    /**
     * Check and execute pause
     */
    bool check_and_execute_pause();

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
     * @return
     */
    bool check_terminate() const;

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
     * @return Whether keyframe is queued or not
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

    //! mutex for access to loop BA related variables
    mutable std::mutex mtx_loop_BA_;

    /**
     * Run loop BA
     * @param lead_keyfrm_id
     */
    void run_loop_BA(unsigned int lead_keyfrm_id);

    //! flag which indicates whether the loop BA is running or not
    bool loop_BA_is_running_ = false;
    //! flag to abort loop BA, which is passed to g2o optimizer
    bool abort_loop_BA_ = false;
    //! thread for running loop BA
    std::unique_ptr<std::thread> thread_for_loop_BA_ = nullptr;

    //! flag which indicates that Sim3 optimization or SE3 optimization is performed
    const bool fix_scale_;

    //! the number of times that loop BA is performed
    unsigned int num_exec_loop_BA_ = 0;
};

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_LOOP_CLOSER_H
