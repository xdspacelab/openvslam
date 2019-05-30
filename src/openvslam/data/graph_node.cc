#include "openvslam/data/keyframe.h"
#include "openvslam/data/graph_node.h"
#include "openvslam/data/landmark.h"

namespace openvslam {
namespace data {

graph_node::graph_node(data::keyframe* keyfrm, const bool is_first_connection)
        : owner_keyfrm_(keyfrm), spanning_parent_is_not_set_(is_first_connection) {}

void graph_node::add_connection(keyframe* keyfrm, const unsigned int weight) {
    bool need_update = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!connected_keyfrms_and_weights_.count(keyfrm)) {
            // なければ追加
            connected_keyfrms_and_weights_[keyfrm] = weight;
            need_update = true;
        }
        else if (connected_keyfrms_and_weights_.at(keyfrm) != weight) {
            // weightが変わっていれば更新
            connected_keyfrms_and_weights_.at(keyfrm) = weight;
            need_update = true;
        }
    }

    // 追加・更新が行われた場合のみグラフの更新が必要
    if (need_update) {
        update_covisibility_orders();
    }
}

void graph_node::erase_connection(keyframe* keyfrm) {
    bool need_update = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (connected_keyfrms_and_weights_.count(keyfrm)) {
            connected_keyfrms_and_weights_.erase(keyfrm);
            need_update = true;
        }
    }

    // 削除が行われた場合のみグラフの更新が必要
    if (need_update) {
        update_covisibility_orders();
    }
}

void graph_node::erase_all_connections() {
    // graphのconnectionを削除する
    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        keyfrm_and_weight.first->erase_connection(owner_keyfrm_);
    }
}

void graph_node::update_connections() {
    const auto landmarks = owner_keyfrm_->get_landmarks();

    std::map<keyframe*, unsigned int> keyfrm_weights;
    for (const auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        const auto observations = lm->get_observations();

        for (const auto& obs : observations) {
            // obs.first: keyframe
            // obs.second: keypoint idx in obs.first
            if (*obs.first == *owner_keyfrm_) {
                continue;
            }

            // count up weight of obs.first
            keyfrm_weights[obs.first]++;
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    unsigned int max_weight = 0;
    keyframe* nearest_covisibility = nullptr;

    // ソートのためのvectorを作る
    std::vector<std::pair<unsigned int, keyframe*>> weight_keyfrm_pairs;
    weight_keyfrm_pairs.reserve(keyfrm_weights.size());
    for (const auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        // nearest covisibilityの情報を更新
        if (max_weight <= weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }

        // covisibilityの情報を更新
        if (weight_thr_ < weight) {
            weight_keyfrm_pairs.emplace_back(std::make_pair(weight, keyfrm));
            // 相手側からもグラフを張る
            keyfrm->add_connection(owner_keyfrm_, weight);
        }
    }

    // 最低一つはweight_keyfrm_pairsに入れておく
    if (weight_keyfrm_pairs.empty()) {
        weight_keyfrm_pairs.emplace_back(std::make_pair(max_weight, nearest_covisibility));
        nearest_covisibility->add_connection(owner_keyfrm_, max_weight);
    }

    // weightの降順でソート
    std::sort(weight_keyfrm_pairs.rbegin(), weight_keyfrm_pairs.rend());

    // 結果を整形する
    std::vector<keyframe*> ordered_connected_keyfrms;
    ordered_connected_keyfrms.reserve(weight_keyfrm_pairs.size());
    std::vector<unsigned int> ordered_weights;
    ordered_weights.reserve(weight_keyfrm_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_keyfrm_pairs) {
        ordered_connected_keyfrms.push_back(weight_keyfrm_pair.second);
        ordered_weights.push_back(weight_keyfrm_pair.first);
    }

    {
        std::lock_guard<std::mutex> lock(mtx_);

        connected_keyfrms_and_weights_ = keyfrm_weights;
        ordered_covisibilities_ = ordered_connected_keyfrms;
        ordered_weights_ = ordered_weights;

        if (spanning_parent_is_not_set_ && owner_keyfrm_->id_ != 0) {
            // nearest covisibilityをspanning treeのparentとする
            assert(*nearest_covisibility == *ordered_connected_keyfrms.front());
            spanning_parent_ = nearest_covisibility;
            spanning_parent_->add_spanning_child(owner_keyfrm_);
            spanning_parent_is_not_set_ = false;
        }
    }
}

void graph_node::update_covisibility_orders() {
    std::lock_guard<std::mutex> lock(mtx_);

    std::vector<std::pair<unsigned int, keyframe*>> weight_keyfrm_pairs;
    weight_keyfrm_pairs.reserve(connected_keyfrms_and_weights_.size());

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        weight_keyfrm_pairs.emplace_back(std::make_pair(keyfrm_and_weight.second, keyfrm_and_weight.first));
    }

    // weightで降順ソート
    std::sort(weight_keyfrm_pairs.rbegin(), weight_keyfrm_pairs.rend());

    ordered_covisibilities_.clear();
    ordered_covisibilities_.reserve(weight_keyfrm_pairs.size());
    ordered_weights_.clear();
    ordered_weights_.reserve(weight_keyfrm_pairs.size());
    for (const auto& weight_keyfrm_pair : weight_keyfrm_pairs) {
        ordered_covisibilities_.push_back(weight_keyfrm_pair.second);
        ordered_weights_.push_back(weight_keyfrm_pair.first);
    }
}

std::set<keyframe*> graph_node::get_connected_keyframes() const {
    std::lock_guard<std::mutex> lock(mtx_);
    std::set<keyframe*> keyfrms;

    for (const auto& keyfrm_and_weight : connected_keyfrms_and_weights_) {
        keyfrms.insert(keyfrm_and_weight.first);
    }

    return keyfrms;
}

std::vector<keyframe*> graph_node::get_covisibilities() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return ordered_covisibilities_;
}

std::vector<keyframe*> graph_node::get_top_n_covisibilities(const unsigned int num_covisibilities) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (ordered_covisibilities_.size() < num_covisibilities) {
        return ordered_covisibilities_;
    }
    else {
        return std::vector<keyframe*>(ordered_covisibilities_.begin(), ordered_covisibilities_.begin() + num_covisibilities);
    }
}

std::vector<keyframe*> graph_node::get_covisibilities_over_weight(const unsigned int weight) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (ordered_covisibilities_.empty()) {
        return std::vector<keyframe*>();
    }

    auto itr = std::upper_bound(ordered_weights_.begin(), ordered_weights_.end(), weight, std::greater<unsigned int>());
    if (itr == ordered_weights_.end()) {
        return std::vector<keyframe*>();
    }
    else {
        const auto num = static_cast<unsigned int>(itr - ordered_weights_.begin());
        return std::vector<keyframe*>(ordered_covisibilities_.begin(), ordered_covisibilities_.begin() + num);
    }
}

unsigned int graph_node::get_weight(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (connected_keyfrms_and_weights_.count(keyfrm)) {
        return connected_keyfrms_and_weights_.at(keyfrm);
    }
    else {
        return 0;
    }
}

void graph_node::add_spanning_child(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.insert(keyfrm);
}

void graph_node::erase_spanning_child(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_children_.erase(keyfrm);
}

void graph_node::recover_spanning_connections() {
    std::lock_guard<std::mutex> lock(mtx_);

    // connection情報をクリア
    connected_keyfrms_and_weights_.clear();
    ordered_covisibilities_.clear();
    ordered_weights_.clear();

    // 1. 自身のchildrenについて整合性を合わせる処理

    // このnodeを削除する場合，
    // このnodeをparentとしている他のnodeに新たなparentを割り当てる必要がある
    // その候補を探す
    std::set<keyframe*> parent_candidates;
    parent_candidates.insert(spanning_parent_);

    while (!spanning_children_.empty()) {
        bool max_is_found = false;

        unsigned int max_weight = 0;
        keyframe* max_weight_child = nullptr;
        keyframe* max_weight_parent = nullptr;

        for (const auto spanning_child : spanning_children_) {
            // 自分の各childについて調べる
            if (spanning_child->will_be_erased()) {
                continue;
            }

            // childのconnectionを持ってくる
            auto connections_from_child = spanning_child->get_covisibilities();

            // childとconnectionがあるkeyframes(connection_from_child)と，
            // parentの候補になるkeyframes(parent_candidates)の中で
            //　一致しているものについて， 一番weightが大きいペアを探す
            for (const auto connection_from_child : connections_from_child) {
                for (const auto& parent_candidate : parent_candidates) {
                    // 一致しているかチェック
                    if (connection_from_child->id_ == parent_candidate->id_) {
                        const auto weight = spanning_child->get_weight(connection_from_child);
                        // maxを比較
                        if (max_weight < weight) {
                            // 更新
                            max_weight = weight;
                            max_weight_child = spanning_child;
                            max_weight_parent = connection_from_child;
                            max_is_found = true;
                        }
                    }
                }
            }
        }

        if (max_is_found) {
            // ペアが見つかったらspanning treeを更新
            max_weight_child->change_spanning_parent(max_weight_parent);
            parent_candidates.insert(max_weight_child);
            spanning_children_.erase(max_weight_child);
        }
        else {
            // 見つからなかった場合はこれ以上更新できないのでbreak
            break;
        }
    }

    // まだspanning_children_が残っていたら，自身のparentを新たなparentとして割り当てる
    if (!spanning_children_.empty()) {
        for (const auto spanning_child : spanning_children_) {
            spanning_child->change_spanning_parent(spanning_parent_);
        }
    }

    // 2. 自身のparentについて整合性を合わせる処理

    // 削除するだけでOK
    spanning_parent_->erase_spanning_child(owner_keyfrm_);
}

void graph_node::set_spanning_parent(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    assert(!spanning_parent_);
    spanning_parent_ = keyfrm;
}

void graph_node::change_spanning_parent(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    spanning_parent_ = keyfrm;
    keyfrm->add_spanning_child(owner_keyfrm_);
}

std::set<keyframe*> graph_node::get_spanning_children() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return spanning_children_;
}

keyframe* graph_node::get_spanning_parent() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return spanning_parent_;
}

bool graph_node::has_spanning_child(keyframe* keyfrm) const {
    std::lock_guard<std::mutex> lock(mtx_);
    return static_cast<bool>(spanning_children_.count(keyfrm));
}

void graph_node::add_loop_edge(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);
    owner_keyfrm_->set_not_to_be_erased();
    loop_edges_.insert(keyfrm);
}

std::set<keyframe*> graph_node::get_loop_edges() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return loop_edges_;
}

bool graph_node::has_loop_edge() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return !loop_edges_.empty();
}

} // namespace data
} // namespace openvslam
