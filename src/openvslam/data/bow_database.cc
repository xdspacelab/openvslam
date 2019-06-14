#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/bow_database.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace data {

bow_database::bow_database(bow_vocabulary* bow_vocab) : bow_vocab_(bow_vocab) {
    spdlog::debug("CONSTRUCT: data::bow_database");
}

bow_database::~bow_database() {
    clear();
    spdlog::debug("DESTRUCT: data::bow_database");
}

void bow_database::add_keyframe(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);

    // keyframes_in_node_のうち，対応するノード番号のlistにkeyframeを追加する
    for (const auto& node_id_and_weight : keyfrm->bow_vec_) {
        keyfrms_in_node_[node_id_and_weight.first].push_back(keyfrm);
    }
}

void bow_database::erase_keyframe(keyframe* keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);

    // keyframes_in_node_のうち，対応するノード番号のlistからkeyframeを削除する
    for (const auto& node_id_and_weight : keyfrm->bow_vec_) {
        // first: node ID, second: weight
        if (!static_cast<bool>(keyfrms_in_node_.count(node_id_and_weight.first))) {
            continue;
        }
        // wordを共有しているキーフレームを取得
        auto& keyfrms_in_node = keyfrms_in_node_.at(node_id_and_weight.first);
        // std::list::eraseがイテレータでの削除にしか対応していないのでイテレータで回す
        for (auto itr = keyfrms_in_node.begin(), lend = keyfrms_in_node.end(); itr != lend; itr++) {
            if (*keyfrm == *(*itr)) {
                keyfrms_in_node.erase(itr);
                break;
            }
        }
    }
}

void bow_database::clear() {
    std::lock_guard<std::mutex> lock(mtx_);
    spdlog::info("clear BoW database");
    keyfrms_in_node_.clear();
}

std::vector<keyframe*> bow_database::acquire_loop_candidates(keyframe* qry_keyfrm, const float min_score) {
    std::lock_guard<std::mutex> lock(tmp_mtx_);

    initialize();

    // 1. データベースの全てのキーフレームについて，query_keyframeと共有しているwordの数(=ノードの数)を集計する

    // query_keyframeの近傍は探索の対象から外す
    auto keyfrms_to_reject = qry_keyfrm->graph_node_->get_connected_keyframes();
    keyfrms_to_reject.insert(qry_keyfrm);

    // 候補がなければ終了
    if (!set_candidates_sharing_words(qry_keyfrm, keyfrms_to_reject)) {
        return std::vector<keyframe*>();
    }

    // 最大共有word数の80%を候補キーフレーム選出の際の最小word数とする
    // (共有ワード数が最大共有word数の80%未満のものはループ候補から除外する)
    unsigned int max_num_common_words = 0;
    for (const auto& candidate : init_candidates_) {
        if (max_num_common_words < num_common_words_.at(candidate)) {
            max_num_common_words = num_common_words_.at(candidate);
        }
    }
    const auto min_num_common_words = static_cast<unsigned int>(0.8f * max_num_common_words);

    // 2. 最小word数(min_num_common_words)より共有word数が多いキーフレーム候補を集める
    //    各キーフレーム候補とquery keyframeの類似度スコアを計算し，min_score以上のものを保持しておく

    // 候補がなければ終了
    if (!compute_scores(qry_keyfrm, min_num_common_words)) {
        return std::vector<keyframe*>();
    }

    // 候補がなければ終了
    if (!align_scores_and_keyframes(min_num_common_words, min_score)) {
        return std::vector<keyframe*>();
    }

    // 3. 各候補キーフレーム(score_keyfrm_pairs)の近傍ともスコアを計算して総和をとる
    //    近傍中で一番スコアが高いものをループ候補にする

    const auto best_total_score = align_total_scores_and_keyframes(min_num_common_words, min_score);

    // 4. total scoreが最大値の75%以上のものを最終的な候補とする

    const float min_total_score = 0.75f * best_total_score;
    std::unordered_set<keyframe*> final_candidates;

    for (const auto& total_score_keyfrm : total_score_keyfrm_pairs_) {
        const auto total_score = total_score_keyfrm.first;
        const auto keyfrm = total_score_keyfrm.second;

        if (min_total_score < total_score) {
            final_candidates.insert(keyfrm);
        }
    }

    return std::vector<keyframe*>(final_candidates.begin(), final_candidates.end());
}

std::vector<keyframe*> bow_database::acquire_relocalization_candidates(frame* qry_frm) {
    std::lock_guard<std::mutex> lock(tmp_mtx_);

    initialize();

    // 1. データベースの全てのキーフレームについて，query frameと共有しているwordの数(=ノードの数)を集計する

    // 候補がなければ終了
    if (!set_candidates_sharing_words(qry_frm)) {
        return std::vector<keyframe*>();
    }

    // 最大共有word数の80%を候補キーフレーム選出の際の最小word数とする
    // (共有ワード数が最大共有word数の80%未満のものはループ候補から除外する)
    unsigned int max_num_common_words = 0;
    for (const auto& candidate : init_candidates_) {
        if (max_num_common_words < num_common_words_.at(candidate)) {
            max_num_common_words = num_common_words_.at(candidate);
        }
    }
    const auto min_num_common_words = static_cast<unsigned int>(0.8f * max_num_common_words);

    // 2. 最小word数(min_num_common_words)より共有word数が多いキーフレーム候補を集める
    //    各キーフレーム候補とquery frameの類似度スコアを計算し，min_score以上のものを保持しておく

    // 候補がなければ終了
    if (!compute_scores(qry_frm, min_num_common_words)) {
        return std::vector<keyframe*>();
    }

    // 候補がなければ終了
    if (!align_scores_and_keyframes(min_num_common_words, 0.0)) {
        return std::vector<keyframe*>();
    }

    // 3. 各候補キーフレーム(score_keyfrm_pairs)の近傍ともスコアを計算して総和をとる
    //    近傍中で一番スコアが高いものをループ候補にする

    const auto best_total_score = align_total_scores_and_keyframes(min_num_common_words, 0.0);

    // 4. total scoreが最大値の75%以上のものを最終的な候補とする

    const float min_total_score = 0.75f * best_total_score;
    std::unordered_set<keyframe*> final_candidates;

    for (const auto& total_score_keyfrm : total_score_keyfrm_pairs_) {
        const auto total_score = total_score_keyfrm.first;
        const auto keyfrm = total_score_keyfrm.second;

        if (min_total_score < total_score) {
            final_candidates.insert(keyfrm);
        }
    }

    return std::vector<keyframe*>(final_candidates.begin(), final_candidates.end());
}

void bow_database::initialize() {
    init_candidates_.clear();
    num_common_words_.clear();
    scores_.clear();
    score_keyfrm_pairs_.clear();
    total_score_keyfrm_pairs_.clear();
}

template<typename T>
bool bow_database::set_candidates_sharing_words(const T* const qry_shot, const std::set<keyframe*>& keyfrms_to_reject) {
    init_candidates_.clear();
    num_common_words_.clear();

    std::lock_guard<std::mutex> lock(mtx_);

    // queryのwords(ノード番号)を取得
    const auto& bow_vec = qry_shot->bow_vec_;
    // queryとwordを共有しているキーフレームについて，共有しているwordの数を集計する
    for (const auto& node_id_and_weight : bow_vec) {
        // first: node ID, second: weight
        // データベースになければcontinue
        if (!static_cast<bool>(keyfrms_in_node_.count(node_id_and_weight.first))) {
            continue;
        }
        // word(ノードID)を共有しているキーフレームを取得
        const auto& keyfrms_in_node = keyfrms_in_node_.at(node_id_and_weight.first);
        // 各キーフレームについて，共有word数を１つずつ増やす
        for (const auto& keyfrm_in_node : keyfrms_in_node) {
            // num_common_wordsに登録されていなかったら初期化する
            if (!static_cast<bool>(num_common_words_.count(keyfrm_in_node))) {
                num_common_words_[keyfrm_in_node] = 0;
                // queryとqueryの近傍以外の場合，第1段階のループ候補(init_loop_candidates)として保存しておく
                if (!static_cast<bool>(keyfrms_to_reject.count(keyfrm_in_node))) {
                    init_candidates_.insert(keyfrm_in_node);
                }
            }
            // word数を増やす
            ++num_common_words_.at(keyfrm_in_node);
        }
    }

    return !init_candidates_.empty();
}

template<typename T>
bool bow_database::compute_scores(const T* const qry_shot, const unsigned int min_num_common_words_thr) {
    scores_.clear();

    for (const auto& candidate : init_candidates_) {
        if (min_num_common_words_thr < num_common_words_.at(candidate)) {
            // 最小共有word数より共有word数が多いキーフレームとqueryの類似度を計算する
#ifdef USE_DBOW2
            const float score = bow_vocab_->score(qry_shot->bow_vec_, candidate->bow_vec_);
#else
            const float score = fbow::BoWVector::score(qry_shot->bow_vec_, candidate->bow_vec_);
#endif
            // スコアを保存
            scores_[candidate] = score;
        }
    }

    return !scores_.empty();
}

bool bow_database::align_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score) {
    score_keyfrm_pairs_.clear();

    // 最小スコアを超えていれば保存 -> score_keyfrm_pairs
    for (const auto& candidate : init_candidates_) {
        if (min_num_common_words_thr < num_common_words_.at(candidate)) {
            const float score = scores_.at(candidate);
            if (min_score <= score) {
                score_keyfrm_pairs_.emplace_back(std::make_pair(score, candidate));
            }
        }
    }

    return !score_keyfrm_pairs_.empty();
}

float bow_database::align_total_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score) {
    total_score_keyfrm_pairs_.clear();

    float best_total_score = min_score;

    for (const auto& score_keyframe : score_keyfrm_pairs_) {
        const auto score = score_keyframe.first;
        const auto keyfrm = score_keyframe.second;

        // keyframeの近傍を取得
        const auto top_n_covisibilities = keyfrm->graph_node_->get_top_n_covisibilities(10);
        // 近傍とのスコアの総和を取る
        // (covisibility_keyframesにkeyframeは含まれないので，scoreで初期化しておく)
        float total_score = score;

        // 候補キーフレームの近傍のうち，最もqueryとの類似度スコアが大きいキーフレームを探す
        float best_score = score;
        auto best_keyframe = keyfrm;

        for (const auto& covisibility : top_n_covisibilities) {
            // 最初のループ候補に含まれており，かつ最小共有word数の条件を満たすものが対象
            if (static_cast<bool>(init_candidates_.count(covisibility))
                && min_num_common_words_thr < num_common_words_.at(covisibility)) {
                // scoreは計算済み
                total_score += scores_.at(covisibility);
                if (best_score < scores_.at(covisibility)) {
                    best_score = scores_.at(covisibility);
                    best_keyframe = covisibility;
                }
            }
        }

        total_score_keyfrm_pairs_.emplace_back(std::make_pair(total_score, best_keyframe));

        if (best_total_score < total_score) {
            best_total_score = total_score;
        }
    }

    return best_total_score;
}

} // namespace data
} // namespace openvslam
