#ifndef OPENVSLAM_DATA_GRAPH_NODE_H
#define OPENVSLAM_DATA_GRAPH_NODE_H

#include <mutex>
#include <vector>
#include <map>
#include <set>

namespace openvslam {
namespace data {

class keyframe;

class graph_node {
public:
    explicit graph_node(data::keyframe* keyfrm);

    ~graph_node() = default;

    //! add connection with weight between this and specified keyframes
    void add_connection(keyframe* keyfrm, const unsigned int weight);

    //! erase connection between this and specified keyframes
    void erase_connection(keyframe* keyfrm);

    //! 3次元点を参照しなおして，connectionとcovisibility graphの情報を作りなおす (新たにkeyframeが追加されるかも)
    void update_connections();

    //! 現在のcovisibility graphは保ったまま，orderの更新のみを行う (新たなkeyframeは追加されない)
    void update_covisibility_orders();

    //! 隣接しているkeyframeを取得する (最小閾値無し)
    std::set<keyframe*> get_connected_keyframes() const;

    //! covisibility keyframesを取得する (最小閾値有り)
    std::vector<keyframe*> get_covisibilities() const;

    //! weightの上位n個のcovisibility keyframesを取得する
    std::vector<keyframe*> get_top_n_covisibilities(const unsigned int num_covisibilities) const;

    //! weight以上のcovisibility keyframesを取得する
    std::vector<keyframe*> get_covisibilities_over_weight(const unsigned int weight) const;

    //! get weight between this and specified keyframe
    unsigned int get_weight(keyframe* keyfrm) const;

    //! add child node of spanning tree
    void add_spanning_child(keyframe* keyfrm);

    //! erase child node of spanning tree
    void erase_spanning_child(keyframe* keyfrm);

    //! set parent node of spanning tree (only used for map loading)
    void set_spanning_parent(keyframe* keyfrm);

    //! change parent node of spanning tree
    void change_spanning_parent(keyframe* keyfrm);

    //! get children of spanning tree
    std::set<keyframe*> get_spanning_children() const;

    //! get parent of spanning tree
    keyframe* get_spanning_parent() const;

    //! whether this keyframe has child or not
    bool has_spanning_child(keyframe* keyfrm) const;

    //! add loop edge
    void add_loop_edge(keyframe* keyfrm);

    //! get loop edges
    std::set<keyframe*> get_loop_edges() const;

private:
    //! keyframe of this node
    data::keyframe* const owner_keyfrm_;

    //! すべての隣接するkeyframeとその間のweightを保存したもの (最小閾値無し)
    std::map<keyframe*, unsigned int> connected_keyfrms_and_weights_;

    //! covisibility graphの閾値
    static constexpr unsigned int weight_thr_ = 15;
    //! 最小閾値を超えたcovisibilityをweight順に並び替えたもの
    std::vector<keyframe*> ordered_covisibilities_;
    //!　ordered_connected_keyfrms_に対応するweights
    std::vector<unsigned int> ordered_weights_;

    keyframe* spanning_parent_ = nullptr;
    std::set<keyframe*> spanning_children_;

    std::set<keyframe*> loop_edges_;

    bool is_first_connection_ = true;

    //! need mutex for access to connections
    mutable std::mutex mtx_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_GRAPH_NODE_H
