#ifndef OPENVSLAM_MAP_LOCAL_MAPPER_H
#define OPENVSLAM_MAP_LOCAL_MAPPER_H

#include "openvslam/data/keyframe.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/data/map_database.h"
#include "openvslam/track/tracker.h"
#include "openvslam/optimize/local_bundle_adjuster.h"

#include <mutex>
#include <atomic>

namespace openvslam {

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
     * @param map_db
     * @param is_monocular
     */
    local_mapper(data::map_database* map_db, const bool is_monocular);

    /**
     * Destructor
     */
    ~local_mapper();

    /**
     * Set tracker
     * @param tracker
     */
    void set_tracker(track::tracker* tracker);

    /**
     * Set loop closer
     * @param loop_closer
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
     * @param keyfrm
     */
    void queue_keyframe(data::keyframe* keyfrm);

    /**
     * Get the number of queued keyframes
     * @return
     */
    unsigned int get_num_queued_keyframes() const;

    /**
     * Get keyframe acceptability
     * @return
     */
    bool get_keyframe_acceptability() const;

    /**
     * Set keyframe acceptability
     * @param acceptability
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
     * Request to oause the local mapper
     * (NOTE: this function does not wait for reset)
     */
    void request_pause();

    /**
     * Check if the local mapper is requested to be paused or not
     * @return
     */
    bool pause_is_requested() const;

    /**
     * Check if the local mapper is paused or not
     * @return
     */
    bool is_paused() const;

    /**
     * Set the flag to force to run the local mapper
     * @param force_to_run
     * @return
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
     * @return whether the local mapper is terminated or not
     */
    bool is_terminated() const;

    //-----------------------------------------
    // management for local BA

    /**
     * Abort the local BA externally
     * (NOTE: this function does not wait for abort)
     */
    void abort_local_BA();

protected:
    //-----------------------------------------
    // main process

    /**
     * Store a new keyframe to the map database
     */
    void store_new_keyframe();

    /**
     * Create new landmarks using neighbor keyframes
     */
    void create_new_landmarks();

    /**
     * Triangulate landmarks between keyframes 1 and 2
     * @param keyfrm_1
     * @param keyfrm_2
     * @param matches
     */
    void triangulate_with_two_keyframes(data::keyframe* keyfrm_1, data::keyframe* keyfrm_2,
                                        const std::vector<std::pair<unsigned int, unsigned int>>& matches);

    /**
     * Check depth is positive or not (if camera model is equirectangular, always return true)
     * @param pos_w
     * @param rot_cw
     * @param trans_cw
     * @param camera
     * @return
     */
    inline bool check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::base* camera) const;

    /**
     * Check reprojection error is within acceptable threshold
     * @tparam T
     * @param pos_w
     * @param rot_cw
     * @param trans_cw
     * @param camera
     * @param keypt
     * @param x_right
     * @param sigma_sq
     * @param is_stereo
     * @return
     */
    template <typename T>
    inline bool check_reprojection_error(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::base* camera,
                                         const cv::Point_<T>& keypt, const float x_right, const float sigma_sq, const bool is_stereo) const;

    /**
     * Check estimated and actual scale factors are within acceptable threshold
     * @param pos_w
     * @param cam_center_1
     * @param cam_center_2
     * @param ratio_factor
     * @param scale_factor_1
     * @param scale_factor_2
     * @return
     */
    inline bool check_scale_factors(const Vec3_t& pos_w, const Vec3_t& cam_center_1, const Vec3_t& cam_center_2,
                                    const float ratio_factor, const float scale_factor_1, const float scale_factor_2) const;

    /**
     * Remove redundant landmarks
     */
    void remove_redundant_landmarks();

    /**
     * Remove redundant keyframes
     */
    void remove_redundant_keyframes();

    /**
     * Count valid and redundant observations in the specified keyframe
     * @param keyfrm
     * @param num_valid_obs
     * @param num_redundant_obs
     */
    void count_redundant_landmarks(data::keyframe* keyfrm, unsigned int& num_valid_obs, unsigned int& num_redundant_obs) const;

    /**
     * Update the new keyframe
     */
    void update_new_keyframe();

    /**
     * Get first and second order covisibilities of current keyframe
     * @param first_order_thr weight threshold of first order covisibility
     * @param second_order_thr weight threshold of second order covisibility
     * @return
     */
    std::unordered_set<data::keyframe*> get_second_order_covisibilities(const unsigned int first_order_thr,
                                                                        const unsigned int second_order_thr);

    /**
     * Fuse duplicated landmarks between current keyframe and covisibility keyframes
     * @param fuse_tgt_keyfrm
     */
    void fuse_landmark_duplication(const std::unordered_set<data::keyframe*>& fuse_tgt_keyfrms);

    //-----------------------------------------
    // management for reset process

    //! mutex for access to reset procedure
    std::mutex mtx_reset_;

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
    //! flag to force the local mapper to be run
    bool force_to_run_ = false;

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
    track::tracker* tracker_;
    //! loop closer
    loop_closer* loop_closer_;

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
    data::keyframe* cur_keyfrm_;

    //! fresh landmarks to check their redundancy
    std::list<data::landmark*> fresh_landmarks_;
};

bool local_mapper::check_depth_is_positive(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::base* camera) const {
    const auto pos_z = rot_cw.block<1, 3>(2, 0).dot(pos_w) + trans_cw(2);
    return !(camera->model_type_ != camera::model_type_t::Equirectangular && pos_z <= 0);
}

template <typename T>
bool local_mapper::check_reprojection_error(const Vec3_t& pos_w, const Mat33_t& rot_cw, const Vec3_t& trans_cw, camera::base* camera,
                                            const cv::Point_<T>& keypt, const float x_right, const float sigma_sq, const bool is_stereo) const {
    assert(is_stereo ^ (x_right < 0));

    // 有意水準5%のカイ2乗値
    // 自由度n=2
    constexpr float chi_sq_2D = 5.99146;
    // 自由度n=3
    constexpr float chi_sq_3D = 7.81473;

    Vec2_t reproj_in_cur;
    float x_right_in_cur;
    camera->reproject_to_image(rot_cw, trans_cw, pos_w, reproj_in_cur, x_right_in_cur);

    if (is_stereo) {
        const Vec2_t reproj_err = reproj_in_cur - keypt;
        const auto reproj_err_x_right = x_right_in_cur - x_right;
        if ((chi_sq_3D * sigma_sq) < (reproj_err.squaredNorm() + reproj_err_x_right * reproj_err_x_right)) {
            return false;
        }
    }
    else {
        const Vec2_t reproj_err = reproj_in_cur - keypt;
        if ((chi_sq_2D * sigma_sq) < reproj_err.squaredNorm()) {
            return false;
        }
    }

    return true;
}

bool local_mapper::check_scale_factors(const Vec3_t& pos_w, const Vec3_t& cam_center_1, const Vec3_t& cam_center_2,
                                       const float ratio_factor, const float scale_factor_1, const float scale_factor_2) const {
    // 3次元点とカメラの間の距離から求めたスケール比と，特徴点のスケール比が大きく異なる場合は破棄

    const Vec3_t cam_1_to_lm_vec = pos_w - cam_center_1;
    const auto cam_1_to_lm_dist = cam_1_to_lm_vec.norm();

    const Vec3_t cam_2_to_lm_vec = pos_w - cam_center_2;
    const auto cam_2_to_lm_dist = cam_2_to_lm_vec.norm();

    if (cam_1_to_lm_dist == 0 || cam_2_to_lm_dist == 0) {
        return false;
    }

    const auto ratio_dists = cam_2_to_lm_dist / cam_1_to_lm_dist;
    const auto ratio_octave = scale_factor_1 / scale_factor_2;

    if (ratio_factor < ratio_octave / ratio_dists || ratio_factor < ratio_dists / ratio_octave) {
        return false;
    }

    return true;
}

} // namespace map
} // namespace openvslam

#endif // OPENVSLAM_MAP_LOCAL_MAPPER_H
