#ifndef OPENVSLAM_MAPPING_MODULE_H
#define OPENVSLAM_MAPPING_MODULE_H

#include "openvslam/camera/base.h"
#include "openvslam/module/local_map_cleaner.h"
#include "openvslam/optimize/local_bundle_adjuster.h"

#include <mutex>
#include <atomic>
#include <memory>

namespace openvslam {

class tracking_module;
class global_optimization_module;

namespace camera {
class base;
} // namespace camera

namespace data {
class keyframe;
class map_database;
} // namespace data

class mapping_module {
public:
    //! Constructor
    mapping_module(data::map_database* map_db, const bool is_monocular);

    //! Destructor
    ~mapping_module();

    //! Set the tracking module
    void set_tracking_module(tracking_module* tracker);

    //! Set the global optimization module
    void set_global_optimization_module(global_optimization_module* global_optimizer);

    //-----------------------------------------
    // main process

    //! Run main loop of the mapping module
    void run();

    //! Queue a keyframe to process the mapping
    void queue_keyframe(const std::shared_ptr<data::keyframe>& keyfrm);

    //! Get the number of queued keyframes
    unsigned int get_num_queued_keyframes() const;

    //! Get keyframe acceptability
    bool get_keyframe_acceptability() const;

    //! Set keyframe acceptability
    void set_keyframe_acceptability(const bool acceptability);

    //-----------------------------------------
    // management for reset process

    //! Request to reset the mapping module
    //! (NOTE: this function waits for reset)
    void request_reset();

    //-----------------------------------------
    // management for pause process

    //! Request to pause the mapping module
    //! (NOTE: this function does not wait for reset)
    void request_pause();

    //! Check if the mapping module is requested to be paused or not
    bool pause_is_requested() const;

    //! Check if the mapping module is paused or not
    bool is_paused() const;

    //! Set the flag to force to run the mapping module
    bool set_force_to_run(const bool force_to_run);

    //! Resume the mapping module
    void resume();

    //-----------------------------------------
    // management for terminate process

    //! Request to terminate the mapping module
    //! (NOTE: this function does not wait for terminate)
    void request_terminate();

    //! Check if the mapping module is terminated or not
    bool is_terminated() const;

    //-----------------------------------------
    // management for local BA

    //! Abort the local BA externally
    //! (NOTE: this function does not wait for abort)
    void abort_local_BA();

private:
    //-----------------------------------------
    // main process

    //! Create and extend the map with the new keyframe
    void mapping_with_new_keyframe();

    //! Store the new keyframe to the map database
    void store_new_keyframe();

    //! Create new landmarks using neighbor keyframes
    void create_new_landmarks();

    //! Triangulate landmarks between the keyframes 1 and 2
    void triangulate_with_two_keyframes(const std::shared_ptr<data::keyframe>& keyfrm_1, const std::shared_ptr<data::keyframe>& keyfrm_2,
                                        const std::vector<std::pair<unsigned int, unsigned int>>& matches);

    //! Update the new keyframe
    void update_new_keyframe();

    //! Get the first and second order covisibilities of current keyframe
    std::unordered_set<std::shared_ptr<data::keyframe>> get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                                        const unsigned int second_order_thr);

    //! Fuse duplicated landmarks between current keyframe and covisibility keyframes
    void fuse_landmark_duplication(const std::unordered_set<std::shared_ptr<data::keyframe>>& fuse_tgt_keyfrms);

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    mutable std::mutex mtx_reset_;

    //! Check and execute reset
    bool reset_is_requested() const;

    //! Reset the variables
    void reset();

    //! flag which indicates whether reset is requested or not
    bool reset_is_requested_ = false;

    //-----------------------------------------
    // management for pause process

    //! mutex for access to pause procedure
    mutable std::mutex mtx_pause_;

    //! Pause the mapping module
    void pause();

    //! flag which indicates termination is requested or not
    bool pause_is_requested_ = false;
    //! flag which indicates whether the main loop is paused or not
    bool is_paused_ = false;
    //! flag to force the mapping module to be run
    bool force_to_run_ = false;

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
    //! global optimization module
    global_optimization_module* global_optimizer_ = nullptr;

    //! local map cleaner
    std::unique_ptr<module::local_map_cleaner> local_map_cleaner_ = nullptr;

    //-----------------------------------------
    // database

    //! map database
    data::map_database* map_db_ = nullptr;

    //-----------------------------------------
    // keyframe queue

    //! mutex for access to keyframe queue
    mutable std::mutex mtx_keyfrm_queue_;

    //! Check if keyframe is queued
    bool keyframe_is_queued() const;

    //! queue for keyframes
    std::list<std::shared_ptr<data::keyframe>> keyfrms_queue_;

    //-----------------------------------------
    // optimizer

    //! local bundle adjuster
    std::unique_ptr<optimize::local_bundle_adjuster> local_bundle_adjuster_ = nullptr;

    //! bridge flag to abort local BA
    bool abort_local_BA_ = false;

    //-----------------------------------------
    // others

    //! flag which indicates the tracking camera is monocular or not
    const bool is_monocular_;

    //! flag for keyframe acceptability
    std::atomic<bool> keyfrm_acceptability_{true};

    //! current keyframe which is used in the current mapping
    std::shared_ptr<data::keyframe> cur_keyfrm_ = nullptr;
};

} // namespace openvslam

#endif // OPENVSLAM_MAPPING_MODULE_H
