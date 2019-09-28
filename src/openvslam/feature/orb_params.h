#ifndef OPENVSLAM_FEATURE_ORB_PARAMS_H
#define OPENVSLAM_FEATURE_ORB_PARAMS_H

#include <yaml-cpp/yaml.h>

namespace openvslam {
namespace feature {

struct orb_params {
    orb_params() = default;

    //! Constructor
    orb_params(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
               const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
               const std::vector<std::vector<float>>& mask_rects = {});

    //! Constructor
    explicit orb_params(const YAML::Node& yaml_node);

    //! Destructor
    virtual ~orb_params() = default;

    //! Dump parameter values to the standard output
    void show_parameters() const;

    unsigned int max_num_keypts_ = 2000;
    float scale_factor_ = 1.2;
    unsigned int num_levels_ = 8;
    unsigned int ini_fast_thr_ = 20;
    unsigned int min_fast_thr = 7;

    //! A vector of keypoint area represents mask area
    //! Each areas are denoted as form of [x_min / cols, x_max / cols, y_min / rows, y_max / rows]
    std::vector<std::vector<float>> mask_rects_;

    //! Calculate scale factors
    static std::vector<float> calc_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of scale factors
    static std::vector<float> calc_inv_scale_factors(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate squared sigmas at all levels
    static std::vector<float> calc_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);

    //! Calculate inverses of squared sigmas at all levels
    static std::vector<float> calc_inv_level_sigma_sq(const unsigned int num_scale_levels, const float scale_factor);
};

} // namespace feature
} // namespace openvslam

#endif // OPENVSLAM_FEATURE_ORB_PARAMS_H
