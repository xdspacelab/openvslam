#include "openvslam/data/frame.h"
#include "openvslam/initialize/bearing_vector.h"
#include "openvslam/solve/essential_solver.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace initialize {

bearing_vector::bearing_vector(const data::frame& ref_frm,
                               const unsigned int num_ransac_iters, const unsigned int min_num_triangulated,
                               const float parallax_deg_thr, const float reproj_err_thr)
    : base(ref_frm, num_ransac_iters, min_num_triangulated, parallax_deg_thr, reproj_err_thr) {
    spdlog::debug("CONSTRUCT: initialize::bearing_vector");
}

bearing_vector::~bearing_vector() {
    spdlog::debug("DESTRUCT: initialize::bearing_vector");
}

bool bearing_vector::initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {
    // set the current camera model
    cur_camera_ = cur_frm.camera_;
    // store the keypoints and bearings
    cur_undist_keypts_ = cur_frm.undist_keypts_;
    cur_bearings_ = cur_frm.bearings_;
    // align matching information
    ref_cur_matches_.clear();
    ref_cur_matches_.reserve(cur_frm.undist_keypts_.size());
    for (unsigned int ref_idx = 0; ref_idx < ref_matches_with_cur.size(); ++ref_idx) {
        const auto cur_idx = ref_matches_with_cur.at(ref_idx);
        if (0 <= cur_idx) {
            ref_cur_matches_.emplace_back(std::make_pair(ref_idx, cur_idx));
        }
    }

    // compute an E matrix
    auto essential_solver = solve::essential_solver(ref_bearings_, cur_bearings_, ref_cur_matches_);
    essential_solver.find_via_ransac(num_ransac_iters_);

    // reconstruct map if the solution is valid
    if (essential_solver.solution_is_valid()) {
        const Mat33_t E_ref_to_cur = essential_solver.get_best_E_21();
        const auto is_inlier_match = essential_solver.get_inlier_matches();
        return reconstruct_with_E(E_ref_to_cur, is_inlier_match);
    }
    else {
        return false;
    }
}

bool bearing_vector::reconstruct_with_E(const Mat33_t& E_ref_to_cur, const std::vector<bool>& is_inlier_match) {
    // found the most plausible pose from the FOUR hypothesis computed from the E matrix

    // decompose the E matrix
    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    if (!solve::essential_solver::decompose(E_ref_to_cur, init_rots, init_transes)) {
        return false;
    }

    assert(init_rots.size() == 4);
    assert(init_transes.size() == 4);

    const auto pose_is_found = find_most_plausible_pose(init_rots, init_transes, is_inlier_match, false);
    if (!pose_is_found) {
        return false;
    }

    spdlog::info("initialization succeeded with E");
    return true;
}

} // namespace initialize
} // namespace openvslam
