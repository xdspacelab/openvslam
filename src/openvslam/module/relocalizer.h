#ifndef OPENVSLAM_MODULE_RELOCALIZER_H
#define OPENVSLAM_MODULE_RELOCALIZER_H

#include "openvslam/match/bow_tree.h"
#include "openvslam/match/projection.h"
#include "openvslam/optimize/pose_optimizer.h"

namespace openvslam {

namespace data {
class frame;
class bow_database;
} // namespace data

namespace module {

class relocalizer {
public:
    /**
     * Constructor
     */
    explicit relocalizer(data::bow_database* bow_db,
                         const double bow_match_lowe_ratio = 0.75, const double proj_match_lowe_ratio = 0.9,
                         const unsigned int min_num_bow_matches = 20, const unsigned int min_num_valid_obs = 50);

    /**
     * Destructor
     */
    ~relocalizer();

    /**
     * Relocalize the specified frame
     */
    bool relocalize(data::frame& curr_frm);

private:
    data::bow_database* bow_db_;

    const unsigned int min_num_bow_matches_;
    const unsigned int min_num_valid_obs_;

    const match::bow_tree bow_matcher_;
    const match::projection proj_matcher_;

    const optimize::pose_optimizer pose_optimizer_;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_RELOCALIZER_H
