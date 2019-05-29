#ifndef OPENVSLAM_DATA_KEYFRAME_H
#define OPENVSLAM_DATA_KEYFRAME_H

#include "openvslam/type.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/bow_vocabulary.h"

#include <set>
#include <mutex>
#include <atomic>

#include <g2o/types/sba/types_six_dof_expmap.h>
#include <nlohmann/json.hpp>

#ifdef USE_DBOW2
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#else
#include <fbow/fbow.h>
#endif

namespace openvslam {

// camera
namespace camera {
class base;
}

namespace data {

class frame;
class landmark;
class map_database;
class bow_database;

class keyframe {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! constructor with frame
    keyframe(const frame& frm, map_database* map_db, bow_database* bow_db);

    //! constructor for map loading with computing parameters which can be recomputed
    keyframe(const unsigned int id, const unsigned int src_frm_id, const double timestamp,
             const Mat44_t& cam_pose_cw, camera::base* camera, const float depth_thr,
             const unsigned int num_keypts, const std::vector<cv::KeyPoint>& keypts,
             const std::vector<cv::KeyPoint>& undist_keypts, const eigen_alloc_vector<Vec3_t>& bearings,
             const std::vector<float>& stereo_x_right, const std::vector<float>& depths, const cv::Mat& descriptors,
             const unsigned int num_scale_levels, const float scale_factor,
             bow_vocabulary* bow_vocab, bow_database* bow_db, map_database* map_db);

    //! operator overrides
    inline bool operator==(const keyframe& keyfrm) const { return id_ == keyfrm.id_; }
    inline bool operator!=(const keyframe& keyfrm) const { return !(*this == keyfrm); }
    inline bool operator<(const keyframe& keyfrm) const { return id_ < keyfrm.id_; }
    inline bool operator<=(const keyframe& keyfrm) const { return id_ <= keyfrm.id_; }
    inline bool operator>(const keyframe& keyfrm) const { return id_ > keyfrm.id_; }
    inline bool operator>=(const keyframe& keyfrm) const { return id_ >= keyfrm.id_; }

    // pose functions with mutual exclusion
    void set_cam_pose(const Mat44_t& cam_pose_cw);
    void set_cam_pose(const g2o::SE3Quat& cam_pose_cw);
    Mat44_t get_cam_pose() const;
    Mat44_t get_cam_pose_inv() const;
    Vec3_t get_cam_center() const;
    Mat33_t get_rotation() const;
    Vec3_t get_translation() const;

    //! compute BoW representation
    void compute_bow();

    // covisibility graphを扱う関数
    //! add connection with weight between this and specified keyframes
    void add_connection(keyframe* keyfrm, const unsigned int weight);
    //! erase connection between this and specified keyframes
    void erase_connection(keyframe* keyfrm);
    //! 3次元点を参照しなおして，connectionとcovisibility graphの情報を作りなおす (新たにkeyframeが追加されるかも)
    void update_connections();
    //! 現在のcovisibility graphは保ったまま，orderの更新のみを行う (新たなkeyframeは追加されない)
    void update_covisibility_orders();
    //! 隣接しているkeyframeを取得する (最小閾値無し)
    std::set<keyframe*> get_connected_keyframes() const;
    //! covisibility keyframesを取得する (最小閾値有り)
    std::vector<keyframe*> get_covisibilities() const;
    //! weightの上位n個のcovisibility keyframesを取得する
    std::vector<keyframe*> get_top_n_covisibilities(const unsigned int num_covisibilities) const;
    //! weight以上のcovisibility keyframesを取得する
    std::vector<keyframe*> get_covisibilities_over_weight(const unsigned int weight) const;
    //! get weight between this and specified keyframe
    unsigned int get_weight(keyframe* keyfrm) const;

    // landmarkを扱う関数
    //! add landmark observed as keyframe at idx
    void add_landmark(landmark* lm, const unsigned int idx);
    //! erase landmark observed as keyframe at idx
    void erase_landmark_with_index(const unsigned int idx);
    //! erase landmark
    void erase_landmark(landmark* lm);
    //! replace landmark
    void replace_landmark(const unsigned int idx, landmark* lm);
    //! get landmarks including nullptr
    std::vector<landmark*> get_landmarks() const;
    //! get landmarks which are not nullptr
    std::set<landmark*> get_valid_landmarks() const;
    //! 最小n個のキーフレームから観測されているlandmarksの数を取得する
    unsigned int get_n_tracked_landmarks(const unsigned int min_num_obs) const;
    //! landmark associated keyframe idx
    landmark* get_landmark(const unsigned int idx) const;

    // spanning treeを扱う関数
    //! add child node of spanning tree
    void add_spanning_child(keyframe* keyfrm);
    //! erase child node of spanning tree
    void erase_spanning_child(keyframe* keyfrm);
    //! set parent node of spanning tree (only used for map loading)
    void set_spanning_parent(keyframe* keyfrm);
    //! change parent node of spanning tree
    void change_spanning_parent(keyframe* keyfrm);
    //! get children of spanning tree
    std::set<keyframe*> get_spanning_children() const;
    //! get parent of spanning tree
    keyframe* get_spanning_parent() const;
    //! whether this keyframe has child or not
    bool has_spanning_child(keyframe* keyfrm) const;

    // loopを扱う関数
    //! add loop edge
    void add_loop_edge(keyframe* keyfrm);
    //! get loop edges
    std::set<keyframe*> get_loop_edges() const;

    // 削除可能かどうかを設定する関数
    //! set this keyframe as non-erasable
    void set_not_to_be_erased();
    //! set this keyframe as erasable
    void set_to_be_erased();

    //! erase this keyframe from database
    void prepare_for_erasing();
    //! whether this keyframe will be erased shortly or not
    bool will_be_erased();

    //! get keypoint indices in the cell which reference point is located
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin) const;

    //! perform stereo triangulation of the keypoint
    Vec3_t triangulate_stereo(const unsigned int idx) const;

    //! compute median of depth
    float compute_median_depth(const bool abs = false) const;

    //! encode keyframe information as JSON
    nlohmann::json to_json() const;

public:
    unsigned int id_;
    static std::atomic<unsigned int> next_id_;

    const unsigned int src_frm_id_;

    const double timestamp_;

    // local mapの更新の際に重複を避けるために用いられる変数
    unsigned int identifier_in_local_map_update_ = 0;

    // loop BAの際に姿勢伝播を行うために用いられる関数
    Mat44_t cam_pose_cw_after_loop_BA_;
    Mat44_t cam_pose_cw_before_BA_;
    unsigned int loop_BA_identifier_ = 0;

    //! camera model
    camera::base* camera_;
    //! depth threshold
    const float depth_thr_;

    //! number of keypoints
    const unsigned int num_keypts_;

    //! keypoints of monocular or stereo left image
    const std::vector<cv::KeyPoint> keypts_;
    //! undistorted keypoints of monocular or stereo left image
    const std::vector<cv::KeyPoint> undist_keypts_;
    //! bearing vectors
    const eigen_alloc_vector<Vec3_t> bearings_;

    //! disparities
    const std::vector<float> stereo_x_right_;
    //! depths
    const std::vector<float> depths_;

    //! descriptors
    const cv::Mat descriptors_;

    //! BoW features (DBoW2 or FBoW)
#ifdef USE_DBOW2
    DBoW2::BowVector bow_vec_;
    DBoW2::FeatureVector bow_feat_vec_;
#else
    fbow::BoWVector bow_vec_;
    fbow::BoWFeatVector bow_feat_vec_;
#endif

    // ORB scale pyramid information
    //! number of scale levels
    const unsigned int num_scale_levels_;
    //! scale factor
    const float scale_factor_;
    //! log scale factor
    const float log_scale_factor_;
    //! list of scale factors
    const std::vector<float> scale_factors_;
    //! list of sigma^2 (sigma=1.0 at scale=0) for optimization
    const std::vector<float> level_sigma_sq_;
    //! list of 1 / sigma^2 for optimization
    const std::vector<float> inv_level_sigma_sq_;

private:
    //-----------------------------------------
    //! need mutex for access to poses
    mutable std::mutex mtx_pose_;

    Mat44_t cam_pose_cw_;
    Mat44_t cam_pose_wc_;
    Vec3_t cam_center_;

    //-----------------------------------------
    //! need mutex for access to observations
    mutable std::mutex mtx_observations_;

    std::vector<landmark*> landmarks_;

    bow_vocabulary* bow_vocab_;
    bow_database* bow_db_;

    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells_;

    //-----------------------------------------
    //! need mutex for access to connections
    mutable std::mutex mtx_connections_;

    //! すべての隣接するkeyframeとその間のweightを保存したもの (最小閾値無し)
    std::map<keyframe*, unsigned int> connected_keyfrms_and_weights_;
    //! 最小閾値を超えたcovisibilityをweight順に並び替えたもの
    std::vector<keyframe*> ordered_connected_keyfrms_;
    //!　ordered_connected_keyfrms_に対応するweights
    std::vector<unsigned int> ordered_weights_;

    bool is_first_connection_ = true;
    keyframe* spanning_parent_ = nullptr;
    std::set<keyframe*> spanning_children_;
    std::set<keyframe*> loop_edges_;

    bool cannot_be_erased_ = false;
    bool prepare_for_erasing_ = false;

    bool will_be_erased_ = false;

    //-----------------------------------------
    //! map database
    map_database* map_db_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_KEYFRAME_H
