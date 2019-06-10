#include "openvslam/data/frame.h"
#include "openvslam/initialize/base.h"

namespace openvslam {
namespace initialize {

base::base(const data::frame& ref_frm, const unsigned int max_num_iters)
        : ref_camera_(ref_frm.camera_), ref_undist_keypts_(ref_frm.undist_keypts_), ref_bearings_(ref_frm.bearings_),
          max_num_iters_(max_num_iters) {}

Mat33_t base::get_rotation_ref_to_cur() const {
    return rot_ref_to_cur_;
}

Vec3_t base::get_translation_ref_to_cur() const {
    return trans_ref_to_cur_;
}

eigen_alloc_vector<Vec3_t> base::get_triangulated_pts() const {
    return triangulated_pts_;
}

std::vector<bool> base::get_triangulated_flags() const {
    return is_triangulated_;
}

} // namespace initialize
} // namespace openvslam
