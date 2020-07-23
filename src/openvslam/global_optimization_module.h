#ifndef OPENVSLAM_GLOBAL_OPTIMIZATION_MODULE_H
#define OPENVSLAM_GLOBAL_OPTIMIZATION_MODULE_H

#include "openvslam/type.h"
#include "openvslam/data/bow_vocabulary.h"
#include "openvslam/module/type.h"
#include "openvslam/module/loop_detector.h"
#include "openvslam/module/loop_bundle_adjuster.h"
#include "openvslam/optimize/graph_optimizer.h"

#include <list>
#include <mutex>
#include <thread>
#include <memory>

namespace openvslam {

class tracking_module;
class mapping_module;

namespace data {
class keyframe;
class bow_database;
class map_database;
} // namespace data

class global_optimization_module {
public:
    //! Constructor
    global_optimization_module(data::map_database* map_db, data::bow_database* bow_db, data::bow_vocabulary* bow_vocab, const bool fix_scale);

    //! Destructor
    ~global_optimization_module();

    //! Set the tracking module
    void set_tracking_module(tracking_module* tracker);

    //! Set the mapping module
    void set_mapping_module(mapping_module* mapper);

    //-----------------------------------------
    // interfaces to ON/OFF loop detector

    //! Enable the loop detector
    void enable_loop_detector();

    //! Disable the loop detector
    void disable_loop_detector();

    //! The loop detector is enabled or not
    bool loop_detector_is_enabled() const;

    //-----------------------------------------
    // main process

    //! Run main loop of the global optimization module
    void run();

    //! Queue a keyframe to the BoW database
    void queue_keyframe(data::keyframe* keyfrm);

    //-----------------------------------------
    // management for reset process

    //! Request to reset the global optimization module
    //! (NOTE: this function waits for reset)
    void request_reset();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the global optimization module
    //! (NOTE: this function does not wait for pause)
    void request_pause();

    //! Check if the global optimization module is requested to be paused or not
    bool pause_is_requested() const;

    //! Check if the global optimization module is paused or not
    bool is_paused() const;

    //! Resume the global optimization module
    void resume();

    //-----------------------------------------
    // management for terminate process

    //! Request to terminate the global optimization module
    //! (NOTE: this function does not wait for terminate)
    void request_terminate();

    //! Check if the global optimization module is terminated or not
    bool is_terminated() const;

    //-----------------------------------------
    // management for loop BA

    //! Check if loop BA is running or not
    bool loop_BA_is_running() const;

    //! Abort the loop BA externally
    //! (NOTE: this function does not wait for abort)
    void abort_loop_BA();

private:
    //-----------------------------------------
    // main process

    //! Perform loop closing
    void correct_loop();

    //! Compute Sim3s (world to covisibility) which are prior to loop correction
    module::keyframe_Sim3_pairs_t get_Sim3s_before_loop_correction(const std::vector<data::keyframe*>& neighbors) const;

    //! Compute Sim3s (world to covisibility) which are corrected using the estimated Sim3 of the current keyframe
    module::keyframe_Sim3_pairs_t get_Sim3s_after_loop_correction(const Mat44_t& cam_pose_wc_before_correction, const g2o::Sim3& g2o_Sim3_cw_after_correction,
                                                                  const std::vector<data::keyframe*>& neighbors) const;

    //! Correct the positions of the landmarks which are seen in covisibilities
    void correct_covisibility_landmarks(const module::keyframe_Sim3_pairs_t& Sim3s_nw_before_correction,
                                        const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    //! Correct the camera poses of the covisibilities
    void correct_covisibility_keyframes(const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    //! Detect and replace duplicated landmarks
    void replace_duplicated_landmarks(const std::vector<std::shared_ptr<data::landmark>>& curr_match_lms_observed_in_cand,
                                      const module::keyframe_Sim3_pairs_t& Sim3s_nw_after_correction) const;

    //! Extract the new connections which will be created AFTER loop correction
    std::map<data::keyframe*, std::set<data::keyframe*>> extract_new_connections(const std::vector<data::keyframe*>& covisibilities) const;

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    //! Check and execute reset
    bool reset_is_requested() const;

    //! Reset the global optimization module
    void reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    //! Pause the global optimizer
    void pause();

    //! flag which indicates termination is requested or not
    bool pause_is_requested_ = false;
    //! flag which indicates whether the main loop is paused or not
    bool is_paused_ = false;

    //-----------------------------------------
    // management for terminate process

    //! mutex for access to terminate procedure
    mutable std::mutex mtx_terminate_;

    //! Check if termination is requested or not
    bool terminate_is_requested() const;

    //! Raise the flag which indicates the main loop has been already terminated
    void terminate();

    //! flag which indicates termination is requested or not
    bool terminate_is_requested_ = false;
    //! flag which indicates whether the main loop is terminated or not
    bool is_terminated_ = true;

    //-----------------------------------------
    // modules

    //! tracking module
    tracking_module* tracker_ = nullptr;
    //! mapping module
    mapping_module* mapper_ = nullptr;

    //! loop detector
    std::unique_ptr<module::loop_detector> loop_detector_ = nullptr;
    //! loop bundle adjuster
    std::unique_ptr<module::loop_bundle_adjuster> loop_bundle_adjuster_ = nullptr;

    //-----------------------------------------
    // keyframe queue

    //! mutex for access to keyframe queue
    mutable std::mutex mtx_keyfrm_queue_;

    //! Check if keyframe is queued
    bool keyframe_is_queued() const;

    //! queue for keyframes
    std::list<data::keyframe*> keyfrms_queue_;

    data::keyframe* cur_keyfrm_ = nullptr;

    //-----------------------------------------
    // optimizer

    //! graph optimizer
    std::unique_ptr<optimize::graph_optimizer> graph_optimizer_ = nullptr;

    //-----------------------------------------
    // variables for loop BA

    //! thread for running loop BA
    std::unique_ptr<std::thread> thread_for_loop_BA_ = nullptr;
};

} // namespace openvslam

#endif // OPENVSLAM_GLOBAL_OPTIMIZATION_MODULE_H
