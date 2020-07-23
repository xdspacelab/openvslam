#ifndef OPENVSLAM_MODULE_RELOCALIZER_H
#define OPENVSLAM_MODULE_RELOCALIZER_H

#include "openvslam/match/bow_tree.h"
#include "openvslam/match/projection.h"
#include "openvslam/optimize/pose_optimizer.h"
#include "openvslam/solve/pnp_solver.h"

#include <memory>

namespace openvslam {

namespace data {
class frame;
class bow_database;
} // namespace data

namespace module {

class relocalizer {
public:
    //! Constructor
    explicit relocalizer(data::bow_database* bow_db,
                         const double bow_match_lowe_ratio = 0.75, const double proj_match_lowe_ratio = 0.9,
                         const unsigned int min_num_bow_matches = 20, const unsigned int min_num_valid_obs = 50);

    //! Destructor
    virtual ~relocalizer();

    //! Relocalize the specified frame
    bool relocalize(data::frame& curr_frm);

private:
    //! Extract valid (non-deleted) landmarks from landmark vector
    std::vector<unsigned int> extract_valid_indices(const std::vector<std::shared_ptr<data::landmark>>& landmarks) const;

    //! Setup PnP solver with the specified 2D-3D matches
    std::unique_ptr<solve::pnp_solver> setup_pnp_solver(const std::vector<unsigned int>& valid_indices,
                                                        const eigen_alloc_vector<Vec3_t>& bearings,
                                                        const std::vector<cv::KeyPoint>& keypts,
                                                        const std::vector<std::shared_ptr<data::landmark>>& matched_landmarks,
                                                        const std::vector<float>& scale_factors) const;

    //! BoW database
    data::bow_database* bow_db_;

    //! minimum threshold of the number of BoW matches
    const unsigned int min_num_bow_matches_;
    //! minimum threshold of the number of valid (= inlier after pose optimization) matches
    const unsigned int min_num_valid_obs_;

    //! BoW matcher
    const match::bow_tree bow_matcher_;
    //! projection matcher
    const match::projection proj_matcher_;
    //! pose optimizer
    const optimize::pose_optimizer pose_optimizer_;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_RELOCALIZER_H
