#ifndef OPENVSLAM_OPTIMIZE_G2O_SE3_SHOT_VERTEX_CONTAINER_H
#define OPENVSLAM_OPTIMIZE_G2O_SE3_SHOT_VERTEX_CONTAINER_H

#include "openvslam/type.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"

#include <unordered_map>

namespace openvslam {

namespace data {
class frame;
class keyframe;
} // namespace data

namespace optimize {
namespace internal {
namespace se3 {

class shot_vertex_container {
public:
    //! Constructor
    explicit shot_vertex_container(const unsigned int offset = 0, const unsigned int num_reserve = 50);

    //! Destructor
    virtual ~shot_vertex_container() = default;

    //! Create and return the g2o vertex created from the specified frame
    shot_vertex* create_vertex(data::frame* frm, const bool is_constant);

    //! Create and return the g2o vertex created from the specified keyframe
    shot_vertex* create_vertex(data::keyframe* keyfrm, const bool is_constant);

    //! Create and return the g2o vertex created from shot ID and camera pose
    shot_vertex* create_vertex(const unsigned int id, const Mat44_t& cam_pose_cw, const bool is_constant);

    //! Get vertex corresponding with the specified frame
    shot_vertex* get_vertex(data::frame* frm) const;

    //! Get vertex corresponding with the specified keyframe
    shot_vertex* get_vertex(data::keyframe* keyfrm) const;

    //! Get vertex corresponding with the specified shot (frame/keyframe) ID
    shot_vertex* get_vertex(const unsigned int id) const;

    //! Convert frame ID to vertex ID
    unsigned int get_vertex_id(data::frame* frm) const;

    //! Convert keyframe ID to vertex ID
    unsigned int get_vertex_id(data::keyframe* keyfrm) const;

    //! Convert shot (frame/keyframe) ID to vertex ID
    unsigned int get_vertex_id(unsigned int id) const;

    //! Convert vertex ID to shot (frame/keyframe) ID
    unsigned int get_id(shot_vertex* vtx);

    //! Convert vertex ID to shot (frame/keyframe) ID
    unsigned int get_id(unsigned int vtx_id) const;

    //! Get maximum vertex ID
    unsigned int get_max_vertex_id() const;

    //! Contains the specified keyframe or not
    bool contain(data::keyframe* keyfrm) const;

    // iterators to sweep shot vertices
    using iterator = std::unordered_map<unsigned int, shot_vertex*>::iterator;
    using const_iterator = std::unordered_map<unsigned int, shot_vertex*>::const_iterator;
    iterator begin();
    const_iterator begin() const;
    iterator end();
    const_iterator end() const;

private:
    //! vertex ID = offset + shot (frame/keyframe) ID
    const unsigned int offset_ = 0;

    //! key: shot (frame/keyframe) ID, value: vertex
    std::unordered_map<unsigned int, shot_vertex*> vtx_container_;

    //! max vertex ID
    unsigned int max_vtx_id_ = 0;
};

inline shot_vertex_container::shot_vertex_container(const unsigned int offset, const unsigned int num_reserve)
    : offset_(offset) {
    vtx_container_.reserve(num_reserve);
}

inline shot_vertex* shot_vertex_container::create_vertex(data::frame* frm, const bool is_constant) {
    return create_vertex(frm->id_, frm->cam_pose_cw_, is_constant);
}

inline shot_vertex* shot_vertex_container::create_vertex(data::keyframe* keyfrm, const bool is_constant) {
    return create_vertex(keyfrm->id_, keyfrm->get_cam_pose(), is_constant);
}

inline shot_vertex* shot_vertex_container::create_vertex(const unsigned int id, const Mat44_t& cam_pose_cw, const bool is_constant) {
    // vertexを作成
    const auto vtx_id = offset_ + id;
    auto vtx = new shot_vertex();
    vtx->setId(vtx_id);
    vtx->setEstimate(util::converter::to_g2o_SE3(cam_pose_cw));
    vtx->setFixed(is_constant);
    // databaseに登録
    vtx_container_[id] = vtx;
    // max IDを更新
    if (max_vtx_id_ < vtx_id) {
        max_vtx_id_ = vtx_id;
    }
    // 作成したvertexをreturn
    return vtx;
}

inline shot_vertex* shot_vertex_container::get_vertex(data::frame* frm) const {
    return get_vertex(frm->id_);
}

inline shot_vertex* shot_vertex_container::get_vertex(data::keyframe* keyfrm) const {
    return get_vertex(keyfrm->id_);
}

inline shot_vertex* shot_vertex_container::get_vertex(const unsigned int id) const {
    return vtx_container_.at(id);
}

inline unsigned int shot_vertex_container::get_vertex_id(data::frame* frm) const {
    return get_vertex_id(frm->id_);
}

inline unsigned int shot_vertex_container::get_vertex_id(data::keyframe* keyfrm) const {
    return get_vertex_id(keyfrm->id_);
}

inline unsigned int shot_vertex_container::get_vertex_id(unsigned int id) const {
    return offset_ + id;
}

inline unsigned int shot_vertex_container::get_id(shot_vertex* vtx) {
    return vtx->id() - offset_;
}

inline unsigned int shot_vertex_container::get_id(unsigned int vtx_id) const {
    return vtx_id - offset_;
}

inline unsigned int shot_vertex_container::get_max_vertex_id() const {
    return max_vtx_id_;
}

inline bool shot_vertex_container::contain(data::keyframe* keyfrm) const {
    return 0 != vtx_container_.count(keyfrm->id_);
}

inline shot_vertex_container::iterator shot_vertex_container::begin() {
    return vtx_container_.begin();
}

inline shot_vertex_container::const_iterator shot_vertex_container::begin() const {
    return vtx_container_.begin();
}

inline shot_vertex_container::iterator shot_vertex_container::end() {
    return vtx_container_.end();
}

inline shot_vertex_container::const_iterator shot_vertex_container::end() const {
    return vtx_container_.end();
}

} // namespace se3
} // namespace internal
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_G2O_SE3_SHOT_VERTEX_CONTAINER_H
