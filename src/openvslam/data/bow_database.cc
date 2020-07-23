#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/bow_database.h"

#include <spdlog/spdlog.h>

namespace openvslam {
namespace data {

bow_database::bow_database(bow_vocabulary* bow_vocab)
    : bow_vocab_(bow_vocab) {
    spdlog::debug("CONSTRUCT: data::bow_database");
}

bow_database::~bow_database() {
    clear();
    spdlog::debug("DESTRUCT: data::bow_database");
}

void bow_database::add_keyframe(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);

    // Append keyframe to the corresponding index in keyframes_in_node_ list
    for (const auto& node_id_and_weight : keyfrm->bow_vec_) {
        keyfrms_in_node_[node_id_and_weight.first].push_back(keyfrm);
    }
}

void bow_database::erase_keyframe(const std::shared_ptr<keyframe>& keyfrm) {
    std::lock_guard<std::mutex> lock(mtx_);

    // Delete keyframe from the coresponding index in keyframes_in_node_ list
    for (const auto& node_id_and_weight : keyfrm->bow_vec_) {
        // first: node ID, second: weight
        if (!static_cast<bool>(keyfrms_in_node_.count(node_id_and_weight.first))) {
            continue;
        }
        // Obtain keyframe which shares word
        auto& keyfrms_in_node = keyfrms_in_node_.at(node_id_and_weight.first);

        // std::list::erase only accepts iterator
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

std::vector<std::shared_ptr<keyframe>> bow_database::acquire_loop_candidates(const std::shared_ptr<keyframe>& qry_keyfrm, const float min_score) {
    std::lock_guard<std::mutex> lock(tmp_mtx_);

    initialize();

    // Step 1.
    // Count up the number of nodes, words which are shared with query_keyframe, for all the keyframes in DoW database

    // Not searching near frames of query_keyframe
    auto keyfrms_to_reject = qry_keyfrm->graph_node_->get_connected_keyframes();
    keyfrms_to_reject.insert(qry_keyfrm);

    // If there are no candidates, done
    if (!set_candidates_sharing_words(qry_keyfrm, keyfrms_to_reject)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // Set min_num_common_words as 80 percentile of max_num_common_words
    // for the following selection of candidate keyframes.
    // (Delete frames from candidates if it has less shared words than 80% of the max_num_common_words)
    unsigned int max_num_common_words = 0;
    for (const auto& candidate : init_candidates_) {
        if (max_num_common_words < num_common_words_.at(candidate)) {
            max_num_common_words = num_common_words_.at(candidate);
        }
    }
    const auto min_num_common_words = static_cast<unsigned int>(0.8f * max_num_common_words);

    // Step 2.
    // Collect keyframe candidates which have more shared words than min_num_common_words
    // by calculating similarity score between each candidate and the query keyframe.

    // If there are no candidates, done
    if (!compute_scores(qry_keyfrm, min_num_common_words)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // If there are no candidates, done
    if (!align_scores_and_keyframes(min_num_common_words, min_score)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // Step 3.
    // Calculate sum of the similarity scores for each of score_keyfrm_pairs and the near frames
    // Candidate will be the frame which has the highest similarity score among the near frames

    const auto best_total_score = align_total_scores_and_keyframes(min_num_common_words, min_score);

    // Step 4.
    // Final candidates have larger total score than 75 percentile
    const float min_total_score = 0.75f * best_total_score;
    std::unordered_set<std::shared_ptr<keyframe>> final_candidates;

    for (const auto& total_score_keyfrm : total_score_keyfrm_pairs_) {
        const auto total_score = total_score_keyfrm.first;
        const auto keyfrm = total_score_keyfrm.second;

        if (min_total_score < total_score) {
            final_candidates.insert(keyfrm);
        }
    }

    return std::vector<std::shared_ptr<keyframe>>(final_candidates.begin(), final_candidates.end());
}

std::vector<std::shared_ptr<keyframe>> bow_database::acquire_relocalization_candidates(frame* qry_frm) {
    std::lock_guard<std::mutex> lock(tmp_mtx_);

    initialize();

    // Step 1.
    // Count up the number of nodes, words which are shared with query_keyframe, for all the keyframes in DoW database

    // If there are no candidates, done
    if (!set_candidates_sharing_words(qry_frm)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // Set min_num_common_words as 80 percentile of max_num_common_words
    // for the following selection of candidate keyframes.
    // (Delete frames from candidates if it has less shared words than 80% of the max_num_common_words)
    unsigned int max_num_common_words = 0;
    for (const auto& candidate : init_candidates_) {
        if (max_num_common_words < num_common_words_.at(candidate)) {
            max_num_common_words = num_common_words_.at(candidate);
        }
    }
    const auto min_num_common_words = static_cast<unsigned int>(0.8f * max_num_common_words);

    // Step 2.
    // Collect keyframe candidates which have more shared words than min_num_common_words
    // by calculating similarity score between each candidate and the query keyframe.

    // If there are no candidates, done
    if (!compute_scores(qry_frm, min_num_common_words)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // If there are no candidates, done
    if (!align_scores_and_keyframes(min_num_common_words, 0.0)) {
        return std::vector<std::shared_ptr<keyframe>>();
    }

    // Step 3.
    // Calculate sum of the similarity scores for each of score_keyfrm_pairs and the near frames
    // Candidate will be the frame which has the highest similarity score among the near frames
    const auto best_total_score = align_total_scores_and_keyframes(min_num_common_words, 0.0);

    // Step 4.
    // Final candidates have larger total score than 75 percentile
    const float min_total_score = 0.75f * best_total_score;
    std::unordered_set<std::shared_ptr<keyframe>> final_candidates;

    for (const auto& total_score_keyfrm : total_score_keyfrm_pairs_) {
        const auto total_score = total_score_keyfrm.first;
        const auto keyfrm = total_score_keyfrm.second;

        if (min_total_score < total_score) {
            final_candidates.insert(keyfrm);
        }
    }

    return std::vector<std::shared_ptr<keyframe>>(final_candidates.begin(), final_candidates.end());
}

void bow_database::initialize() {
    init_candidates_.clear();
    num_common_words_.clear();
    scores_.clear();
    score_keyfrm_pairs_.clear();
    total_score_keyfrm_pairs_.clear();
}

template<typename T>
bool bow_database::set_candidates_sharing_words(const T qry_shot, const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject) {
    init_candidates_.clear();
    num_common_words_.clear();

    std::lock_guard<std::mutex> lock(mtx_);

    // Get word (node index) of the query
    const auto& bow_vec = qry_shot->bow_vec_;
    // Count the number of shared words for keyframes which share the word with the query keyframe
    for (const auto& node_id_and_weight : bow_vec) {
        // first: node ID, second: weight
        // If not in the BoW database, continue
        if (!static_cast<bool>(keyfrms_in_node_.count(node_id_and_weight.first))) {
            continue;
        }
        // Get a keyframe which shares the word (node ID) with the query
        const auto& keyfrms_in_node = keyfrms_in_node_.at(node_id_and_weight.first);
        // For each keyframe, increase shared word number one by one
        for (const auto& keyfrm_in_node : keyfrms_in_node) {
            // Initialize if not in num_common_words
            if (!static_cast<bool>(num_common_words_.count(keyfrm_in_node))) {
                num_common_words_[keyfrm_in_node] = 0;
                // If far enough from the query keyframe, store it as the initial loop candidates
                if (!static_cast<bool>(keyfrms_to_reject.count(keyfrm_in_node))) {
                    init_candidates_.insert(keyfrm_in_node);
                }
            }
            // Count up the number of words
            ++num_common_words_.at(keyfrm_in_node);
        }
    }

    return !init_candidates_.empty();
}

template<typename T>
bool bow_database::compute_scores(const T qry_shot, const unsigned int min_num_common_words_thr) {
    scores_.clear();

    for (const auto& candidate : init_candidates_) {
        if (min_num_common_words_thr < num_common_words_.at(candidate)) {
            // Calculate similarity score with query keyframe
            // for the keyframes which have more shared words than minimum common words
#ifdef USE_DBOW2
            const float score = bow_vocab_->score(qry_shot->bow_vec_, candidate->bow_vec_);
#else
            const float score = fbow::BoWVector::score(qry_shot->bow_vec_, candidate->bow_vec_);
#endif
            // Store score
            scores_[candidate] = score;
        }
    }

    return !scores_.empty();
}

bool bow_database::align_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score) {
    score_keyfrm_pairs_.clear();

    // If larger than the minimum score, store to score_keyfrm_pairs
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

        // Get near frames of keyframe
        const auto top_n_covisibilities = keyfrm->graph_node_->get_top_n_covisibilities(10);
        // Calculate the sum of scores for the near frames
        // Initialize with score since keyframe is not included in covisibility_keyframes
        float total_score = score;

        // Find a keyframe which has best similarity score with query keyframe from the near frames
        float best_score = score;
        auto best_keyframe = keyfrm;

        for (const auto& covisibility : top_n_covisibilities) {
            // Loop for which is included in the initial loop candidates and satisfies the minimum shared word number
            if (static_cast<bool>(init_candidates_.count(covisibility))
                && min_num_common_words_thr < num_common_words_.at(covisibility)) {
                // score has already been computed
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
