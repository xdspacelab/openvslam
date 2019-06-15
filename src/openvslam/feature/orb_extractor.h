#ifndef OPENVSLAM_FEATURE_ORB_EXTRACTOR_H
#define OPENVSLAM_FEATURE_ORB_EXTRACTOR_H

#include "openvslam/feature/orb_params.h"
#include "openvslam/feature/orb_extractor_node.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

namespace openvslam {
namespace feature {

class orb_extractor {
public:
    orb_extractor() = delete;

    orb_extractor(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
                  const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
                  const std::vector<std::vector<float>>& mask_rects = {});

    explicit orb_extractor(const orb_params& orb_params);

    virtual ~orb_extractor() = default;

    void extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                 std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors);

    unsigned int get_max_num_keypoints() const;

    void set_max_num_keypoints(const unsigned int max_num_keypts);

    float get_scale_factor() const;

    void set_scale_factor(const float scale_factor);

    unsigned int get_num_scale_levels() const;

    void set_num_scale_levels(const unsigned int num_levels);

    unsigned int get_initial_fast_threshold() const;

    void set_initial_fast_threshold(const unsigned int initial_fast_threshold);

    unsigned int get_minimum_fast_threshold() const;

    void set_minimum_fast_threshold(const unsigned int minimum_fast_threshold);

    std::vector<float> get_scale_factors() const;

    std::vector<float> get_inv_scale_factors() const;

    std::vector<float> get_level_sigma_sq() const;

    std::vector<float> get_inv_level_sigma_sq() const;

    std::vector<cv::Mat> image_pyramid_;

private:
    void initialize();

    void calc_scale_factors();

    void create_rectangle_mask(const unsigned int cols, const unsigned int rows);

    void compute_image_pyramid(const cv::Mat& image);

    /**
     * 画像上の，スケール・グリッドごとに特徴点を計算
     * @param all_keypts
     */
    void compute_fast_keypoints(std::vector<std::vector<cv::KeyPoint>>& all_keypts, const cv::Mat& mask) const;

    /**
     * 大量のFAST特徴点から，FASTのresponseの高いものを画像上からまんべんなく選別する．木構造が選別に用いられる．
     * 再帰的に木の分岐を行い，nodeの数がnum_keyptsを上回った時点の各leaf nodeの最大responseとなる特徴点がreturnされる
     * (分岐方法 - nodeを4分割(縦横2分割)し，4つの子nodeに領域を割り当て，その領域に応じて特徴点を分配)
     * @param keypts_to_distribute
     * @param min_x
     * @param max_x
     * @param min_y
     * @param max_y
     * @param num_keypts
     * @return
     */
    std::vector<cv::KeyPoint> distribute_keypoints_via_tree(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                            const int min_x, const int max_x, const int min_y, const int max_y,
                                                            const unsigned int num_keypts) const;

    std::list<orb_extractor_node> initialize_nodes(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                   const int min_x, const int max_x, const int min_y, const int max_y) const;

    void assign_child_nodes(const std::array<orb_extractor_node, 4>& child_nodes, std::list<orb_extractor_node>& nodes,
                            std::vector<std::pair<int, orb_extractor_node*>>& leaf_nodes) const;

    std::vector<cv::KeyPoint> find_keypoints_with_max_response(std::list<orb_extractor_node>& nodes) const;

    void compute_orientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypts) const;

    void correct_keypoint_scale(std::vector<cv::KeyPoint>& keypts_at_level, const unsigned int level) const;

    /**
     * 予め決めた円周上のピクセル輝度を正弦関数で近似し，
     * その位相を計算する
     * @param image
     * @param point
     * @return
     */
    float ic_angle(const cv::Mat& image, const cv::Point2f& point) const;

    void compute_orb_descriptors(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypts, cv::Mat& descriptors) const;

    void compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, uchar* desc) const;

    //! parameters for ORB extraction
    orb_params orb_params_;

    //! BRIEF orientation
    static constexpr unsigned int fast_patch_size_ = 31;
    //! half size of FAST patch
    static constexpr int fast_half_patch_size_ = fast_patch_size_ / 2;

    //! size of maximum ORB patch radius
    static constexpr unsigned int orb_patch_radius_ = 19;

    //! rectangle mask has been already initialized or not
    bool mask_is_initialized_ = false;
    cv::Mat rect_mask_;

    std::vector<float> scale_factors_;
    std::vector<float> inv_scale_factors_;
    std::vector<float> level_sigma_sq_;
    std::vector<float> inv_level_sigma_sq_;

    std::vector<unsigned int> num_keypts_per_level_;
    std::vector<int> u_max_;
};

} // namespace feature
} // namespace openvslam

#endif // OPENVSLAM_FEATURE_ORB_EXTRACTOR_H
