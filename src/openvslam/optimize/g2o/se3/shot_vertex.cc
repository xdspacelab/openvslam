#include "openvslam/optimize/g2o/se3/shot_vertex.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

shot_vertex::shot_vertex()
    : BaseVertex<6, ::g2o::SE3Quat>() {}

bool shot_vertex::read(std::istream& is) {
    Vec7_t estimate;
    for (unsigned int i = 0; i < 7; ++i) {
        is >> estimate(i);
    }
    ::g2o::SE3Quat g2o_cam_pose_wc;
    g2o_cam_pose_wc.fromVector(estimate);
    setEstimate(g2o_cam_pose_wc.inverse());
    return true;
}

bool shot_vertex::write(std::ostream& os) const {
    ::g2o::SE3Quat g2o_cam_pose_wc(estimate().inverse());
    for (unsigned int i = 0; i < 7; ++i) {
        os << g2o_cam_pose_wc[i] << " ";
    }
    return os.good();
}

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
