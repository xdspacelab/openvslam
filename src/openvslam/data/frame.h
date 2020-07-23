#ifndef OPENVSLAM_DATA_FRAME_H
#define OPENVSLAM_DATA_FRAME_H

#include "openvslam/type.h"
#include "openvslam/camera/base.h"
#include "openvslam/util/converter.h"
#include "openvslam/data/bow_vocabulary.h"

#include <vector>
#include <atomic>

#include <opencv2/core.hpp>
#include <Eigen/Core>

#ifdef USE_DBOW2
#include <DBoW2/BowVector.h>
#include <DBoW2/FeatureVector.h>
#else
#include <fbow/fbow.h>
#endif

namespace openvslam {

namespace camera {
class base;
} // namespace camera

namespace feature {
class orb_extractor;
} // namespace feature

namespace data {

class keyframe;
class landmark;

class frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    frame() = default;

    bool operator==(const frame& frm) { return this->id_ == frm.id_; }
    bool operator!=(const frame& frm) { return !(*this == frm); }

    /**
     * Constructor for monocular frame
     * @param img_gray
     * @param timestamp
     * @param extractor
     * @param bow_vocab
     * @param camera
     * @param depth_thr
     * @param mask
     */
    frame(const cv::Mat& img_gray, const double timestamp,
          feature::orb_extractor* extractor, bow_vocabulary* bow_vocab,
          camera::base* camera, const float depth_thr,
          const cv::Mat& mask = cv::Mat{});

    /**
     * Constructor for stereo frame
     * @param left_img_gray
     * @param right_img_gray
     * @param timestamp
     * @param extractor_left
     * @param extractor_right
     * @param bow_vocab
     * @param camera
     * @param depth_thr
     * @param mask
     */
    frame(const cv::Mat& left_img_gray, const cv::Mat& right_img_gray, const double timestamp,
          feature::orb_extractor* extractor_left, feature::orb_extractor* extractor_right, bow_vocabulary* bow_vocab,
          camera::base* camera, const float depth_thr,
          const cv::Mat& mask = cv::Mat{});

    /**
     * Constructor for RGBD frame
     * @param img_gray
     * @param img_depth
     * @param timestamp
     * @param extractor
     * @param bow_vocab
     * @param camera
     * @param depth_thr
     * @param mask
     */
    frame(const cv::Mat& img_gray, const cv::Mat& img_depth, const double timestamp,
          feature::orb_extractor* extractor, bow_vocabulary* bow_vocab,
          camera::base* camera, const float depth_thr,
          const cv::Mat& mask = cv::Mat{});

    /**
     * Set camera pose and refresh rotation and translation
     * @param cam_pose_cw
     */
    void set_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Set camera pose and refresh rotation and translation
     * @param cam_pose_cw
     */
    void set_cam_pose(const g2o::SE3Quat& cam_pose_cw);

    /**
     * Update rotation and translation using cam_pose_cw_
     */
    void update_pose_params();

    /**
     * Get camera center
     * @return
     */
    Vec3_t get_cam_center() const;

    /**
     * Get inverse of rotation
     * @return
     */
    Mat33_t get_rotation_inv() const;

    /**
     * Update ORB information
     */
    void update_orb_info();

    /**
     * Compute BoW representation
     */
    void compute_bow();

    /**
     * Check observability of the landmark
     */
    bool can_observe(const std::shared_ptr<landmark>& lm, const float ray_cos_thr,
                     Vec2_t& reproj, float& x_right, unsigned int& pred_scale_level) const;

    /**
     * Get keypoint indices in the cell which reference point is located
     * @param ref_x
     * @param ref_y
     * @param margin
     * @param min_level
     * @param max_level
     * @return
     */
    std::vector<unsigned int> get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level = -1, const int max_level = -1) const;

    /**
     * Perform stereo triangulation of the keypoint
     * @param idx
     * @return
     */
    Vec3_t triangulate_stereo(const unsigned int idx) const;

    //! current frame ID
    unsigned int id_;

    //! next frame ID
    static std::atomic<unsigned int> next_id_;

    //! BoW vocabulary (DBoW2 or FBoW)
    bow_vocabulary* bow_vocab_ = nullptr;

    // ORB extractor
    //! ORB extractor for monocular or stereo left image
    feature::orb_extractor* extractor_ = nullptr;
    //! ORB extractor for stereo right image
    feature::orb_extractor* extractor_right_ = nullptr;

    //! timestamp
    double timestamp_;

    //! camera model
    camera::base* camera_ = nullptr;

    // if a stereo-triangulated point is farther than this threshold, it is invalid
    //! depth threshold
    float depth_thr_;

    //! number of keypoints
    unsigned int num_keypts_ = 0;

    // keypoints
    //! keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> keypts_;
    //! keypoints of stereo right image
    std::vector<cv::KeyPoint> keypts_right_;
    //! undistorted keypoints of monocular or stereo left image
    std::vector<cv::KeyPoint> undist_keypts_;
    //! bearing vectors
    eigen_alloc_vector<Vec3_t> bearings_;

    //! disparities
    std::vector<float> stereo_x_right_;
    //! depths
    std::vector<float> depths_;

    //! BoW features (DBoW2 or FBoW)
#ifdef USE_DBOW2
    DBoW2::BowVector bow_vec_;
    DBoW2::FeatureVector bow_feat_vec_;
#else
    fbow::BoWVector bow_vec_;
    fbow::BoWFeatVector bow_feat_vec_;
#endif

    // ORB descriptors
    //! ORB descriptors of monocular or stereo left image
    cv::Mat descriptors_;
    //! ORB descriptors of stereo right image
    cv::Mat descriptors_right_;

    //! landmarks, whose nullptr indicates no-association
    std::vector<std::shared_ptr<landmark>> landmarks_;

    //! outlier flags, which are mainly used in pose optimization and bundle adjustment
    std::vector<bool> outlier_flags_;

    //! cells for storing keypoint indices
    std::vector<std::vector<std::vector<unsigned int>>> keypt_indices_in_cells_;

    //! camera pose: world -> camera
    bool cam_pose_cw_is_valid_ = false;
    Mat44_t cam_pose_cw_;

    //! reference keyframe for tracking
    keyframe* ref_keyfrm_ = nullptr;

    // ORB scale pyramid information
    //! number of scale levels
    unsigned int num_scale_levels_;
    //! scale factor
    float scale_factor_;
    //! log scale factor
    float log_scale_factor_;
    //! list of scale factors
    std::vector<float> scale_factors_;
    //! list of inverse of scale factors
    std::vector<float> inv_scale_factors_;
    //! list of sigma^2 (sigma=1.0 at scale=0) for optimization
    std::vector<float> level_sigma_sq_;
    //! list of 1 / sigma^2 for optimization
    std::vector<float> inv_level_sigma_sq_;

private:
    //! enumeration to control the behavior of extract_orb()
    enum class image_side { Left,
                            Right };

    /**
     * Extract ORB feature according to img_size
     * @param img
     * @param mask
     * @param img_side
     */
    void extract_orb(const cv::Mat& img, const cv::Mat& mask, const image_side& img_side = image_side::Left);

    /**
     * Compute disparities from depth information in depthmap
     * @param right_img_depth
     */
    void compute_stereo_from_depth(const cv::Mat& right_img_depth);

    //! Camera pose
    //! rotation: world -> camera
    Mat33_t rot_cw_;
    //! translation: world -> camera
    Vec3_t trans_cw_;
    //! rotation: camera -> world
    Mat33_t rot_wc_;
    //! translation: camera -> world
    Vec3_t cam_center_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_FRAME_H
