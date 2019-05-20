#ifndef OPENVSLAM_FEATURE_ORB_PARAMS_H
#define OPENVSLAM_FEATURE_ORB_PARAMS_H

#include <yaml-cpp/yaml.h>

namespace openvslam {
namespace feature {

struct orb_params {
    orb_params();

    orb_params(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
               const unsigned int edge_thr = 19, const unsigned int patch_size = 31,
               const std::vector<std::vector<float>>& mask_rects = {});

    explicit orb_params(const YAML::Node& yaml_node);

    virtual ~orb_params() = default;

    void show_parameters() const;

    unsigned int max_num_keypts_ = 2000;
    float scale_factor_ = 1.2;
    unsigned int num_levels_ = 8;
    unsigned int ini_fast_thr_ = 20;
    unsigned int min_fast_thr = 7;
    unsigned int edge_thr_ = 19;
    unsigned int patch_size_ = 31;
    int half_patch_size_;

    //! mask領域を表す特徴点領域のvector
    //! 各領域は[x_min / cols, x_max / cols, y_min / rows, y_max / rows]で表現
    std::vector<std::vector<float>> mask_rects_;

    /**
     * Calculate scale factors
     * @param num_scale_levels
     * @param scale_factor
     * @return
     */
    static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    /**
     * Calculate inverses of scale factors
     * @param num_scale_levels
     * @param scale_factor
     * @return
     */
    static std::vector<float> calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    /**
     * Calculate squared sigmas at all levels
     * @param num_scale_levels
     * @param scale_factor
     * @return
     */
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    /**
     * Calculate inverses of squared sigmas at all levels
     * @param num_scale_levels
     * @param scale_factor
     * @return
     */
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
};

} // namespace feature
} // namespace openvslam

#endif // OPENVSLAM_FEATURE_ORB_PARAMS_H
