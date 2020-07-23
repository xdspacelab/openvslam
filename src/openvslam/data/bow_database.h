#ifndef OPENVSLAM_DATA_BOW_DATABASE_H
#define OPENVSLAM_DATA_BOW_DATABASE_H

#include "openvslam/data/bow_vocabulary.h"

#include <mutex>
#include <list>
#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace openvslam {
namespace data {

class frame;
class keyframe;

class bow_database {
public:
    /**
     * Constructor
     * @param bow_vocab
     */
    explicit bow_database(bow_vocabulary* bow_vocab);

    /**
     * Destructor
     */
    ~bow_database();

    /**
     * Add a keyframe to the database
     * @param keyfrm
     */
    void add_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Erase the keyframe from the database
     * @param keyfrm
     */
    void erase_keyframe(const std::shared_ptr<keyframe>& keyfrm);

    /**
     * Clear the database
     */
    void clear();

    /**
     * Acquire loop-closing candidates over the specified score
     * @param qry_keyfrm
     * @param min_score
     * @return
     */
    std::vector<std::shared_ptr<keyframe>> acquire_loop_candidates(const std::shared_ptr<keyframe>& qry_keyfrm, const float min_score);

    /**
     * Acquire relocalization candidates
     * @param qry_frm
     * @return
     */
    std::vector<std::shared_ptr<keyframe>> acquire_relocalization_candidates(frame* qry_frm);

protected:
    /**
     * Initialize temporary variables
     */
    void initialize();

    /**
     * Compute the number of shared words and set candidates (init_candidates_ and num_common_words_)
     * @tparam T
     * @param qry_shot
     * @param keyfrms_to_reject
     * @return whether candidates are found or not
     */
    template<typename T>
    bool set_candidates_sharing_words(const T qry_shot, const std::set<std::shared_ptr<keyframe>>& keyfrms_to_reject = {});

    /**
     * Compute scores (scores_) between the query and the each of keyframes contained in the database
     * @tparam T
     * @param qry_shot
     * @param min_num_common_words_thr
     * @return whether candidates are found or not
     */
    template<typename T>
    bool compute_scores(const T qry_shot, const unsigned int min_num_common_words_thr);

    /**
     * Align scores and keyframes only which have greater scores than the minimum one
     * @param min_num_common_words_thr
     * @param min_score
     * @return whether candidates are found or not
     */
    bool align_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score);

    /**
     * Compute and align total scores and keyframes
     * @param min_num_common_words_thr
     * @param min_score
     * @return
     */
    float align_total_scores_and_keyframes(const unsigned int min_num_common_words_thr, const float min_score);

    //-----------------------------------------
    // BoW feature vectors

    //! mutex to access BoW database
    mutable std::mutex mtx_;
    //! BoW database
    std::unordered_map<unsigned int, std::list<std::shared_ptr<keyframe>>> keyfrms_in_node_;

    //-----------------------------------------
    // BoW vocabulary

    //! BoW vocabulary
    bow_vocabulary* bow_vocab_;

    //-----------------------------------------
    // temporary variables

    //! mutex to access temporary variables
    mutable std::mutex tmp_mtx_;

    //! initial candidates for loop or relocalization
    std::unordered_set<std::shared_ptr<keyframe>> init_candidates_;

    // key: queryとwordを共有しているキーフレーム, value: 共有word数
    //! number of shared words between the query and the each of keyframes contained in the database
    std::unordered_map<std::shared_ptr<keyframe>, unsigned int> num_common_words_;

    // key: queryとwordを共有しているキーフレーム, value: スコア
    //! similarity scores between the query and the each of keyframes contained in the database
    std::unordered_map<std::shared_ptr<keyframe>, float> scores_;

    // min_score以上のものを保存しておくvector (スコア, キーフレーム)
    //! pairs of score and keyframe which has the larger score than the minimum one
    std::vector<std::pair<float, std::shared_ptr<keyframe>>> score_keyfrm_pairs_;

    // min_score以上のものを保存しておくvector (スコア, キーフレーム)
    //! pairs of total score and keyframe which has the larger score than the minimum one
    std::vector<std::pair<float, std::shared_ptr<keyframe>>> total_score_keyfrm_pairs_;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_DATA_BOW_DATABASE_H
