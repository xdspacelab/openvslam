#include "openvslam/feature/orb_extractor.h"
#include "openvslam/feature/orb_point_pairs.h"
#include "openvslam/util/trigonometric.h"

#include <opencv2/opencv.hpp>

namespace openvslam {
namespace feature {

orb_extractor::orb_extractor(const unsigned int max_num_keypts, const float scale_factor, const unsigned int num_levels,
                             const unsigned int ini_fast_thr, const unsigned int min_fast_thr,
                             const unsigned int edge_thr, const unsigned int patch_size,
                             const std::vector<std::vector<float>>& mask_rects)
        : orb_extractor(orb_params{max_num_keypts, scale_factor, num_levels,
                                   ini_fast_thr, min_fast_thr,
                                   edge_thr, patch_size,
                                   mask_rects}) {}

orb_extractor::orb_extractor(const orb_params& orb_params)
        : orb_params_(orb_params) {
    // create ORB point pairs
    create_point_pairs();
    // initialize parameters
    initialize();
}

void orb_extractor::extract(const cv::_InputArray& in_image, const cv::_InputArray& in_image_mask,
                            std::vector<cv::KeyPoint>& keypts, const cv::_OutputArray& out_descriptors) {
    if (in_image.empty()) {
        return;
    }

    // get cv::Mat of image
    const auto image = in_image.getMat();
    assert(image.type() == CV_8UC1);

    // build image pyramid
    compute_image_pyramid(image);

    // mask initialization
    if (!mask_is_initialized_ && !orb_params_.mask_rects_.empty()) {
        create_rectangle_mask(image.cols, image.rows);
        mask_is_initialized_ = true;
    }

    std::vector<std::vector<cv::KeyPoint>> all_keypts;

    // select mask to use
    if (!in_image_mask.empty()) {
        // image_maskが有効であればそちらを使う
        const auto image_mask = in_image_mask.getMat();
        assert(image_mask.type() == CV_8UC1);
        compute_fast_keypoints(all_keypts, image_mask);
    }
    else if (!rect_mask_.empty()){
        // image_maskが無効かつrectangle maskが有効であればrectangle maskを使う
        assert(rect_mask_.type() == CV_8UC1);
        compute_fast_keypoints(all_keypts, rect_mask_);
    }
    else {
        // いずれのmaskも無効であれば使わない
        compute_fast_keypoints(all_keypts, cv::Mat());
    }

    cv::Mat descriptors;

    unsigned int num_keypts = 0;
    for (unsigned int level = 0; level < orb_params_.num_levels_; ++level) {
        num_keypts += all_keypts.at(level).size();
    }
    if (num_keypts == 0) {
        out_descriptors.release();
    }
    else {
        out_descriptors.create(num_keypts, 32, CV_8U);
        descriptors = out_descriptors.getMat();
    }

    keypts.clear();
    keypts.reserve(num_keypts);

    unsigned int offset = 0;
    for (unsigned int level = 0; level < orb_params_.num_levels_; ++level) {
        auto& keypts_at_level = all_keypts.at(level);
        const auto num_keypts_at_level = keypts_at_level.size();

        if (num_keypts_at_level == 0) {
            continue;
        }

        cv::Mat blurred_image = image_pyramid_.at(level).clone();
        cv::GaussianBlur(blurred_image, blurred_image, cv::Size(7, 7), 2, 2, cv::BORDER_REFLECT_101);

        cv::Mat descriptors_at_level = descriptors.rowRange(offset, offset + num_keypts_at_level);
        compute_orb_descriptors(blurred_image, keypts_at_level, descriptors_at_level);

        offset += num_keypts_at_level;

        correct_keypoint_scale(keypts_at_level, level);

        keypts.insert(keypts.end(), keypts_at_level.begin(), keypts_at_level.end());
    }
}

unsigned int orb_extractor::get_max_num_keypoints() const {
    return orb_params_.max_num_keypts_;
}

void orb_extractor::set_max_num_keypoints(const unsigned int max_num_keypts) {
    orb_params_.max_num_keypts_ = max_num_keypts;
    initialize();
}

float orb_extractor::get_scale_factor() const {
    return orb_params_.scale_factor_;
}

void orb_extractor::set_scale_factor(const float scale_factor) {
    orb_params_.scale_factor_ = scale_factor;
    initialize();
}

unsigned int orb_extractor::get_num_scale_levels() const {
    return orb_params_.num_levels_;
}

void orb_extractor::set_num_scale_levels(const unsigned int num_levels) {
    orb_params_.num_levels_ = num_levels;
    initialize();
}

unsigned int orb_extractor::get_initial_fast_threshold() const {
    return orb_params_.ini_fast_thr_;
}

void orb_extractor::set_initial_fast_threshold(const unsigned int initial_fast_threshold) {
    orb_params_.ini_fast_thr_ = initial_fast_threshold;
}

unsigned int orb_extractor::get_minimum_fast_threshold() const {
    return orb_params_.min_fast_thr;
}

void orb_extractor::set_minimum_fast_threshold(const unsigned int minimum_fast_threshold) {
    orb_params_.min_fast_thr = minimum_fast_threshold;
}

unsigned int orb_extractor::get_edge_threshold() const {
    return orb_params_.edge_thr_;
}

void orb_extractor::set_edge_threshold(const unsigned int edge_threshold) {
    orb_params_.edge_thr_ = edge_threshold;
}

unsigned int orb_extractor::get_patch_size() const {
    return orb_params_.patch_size_;
}

void orb_extractor::set_patch_size(const unsigned int patch_size) {
    orb_params_.patch_size_ = patch_size;
    initialize();
}

std::vector<float> orb_extractor::get_scale_factors() const {
    return scale_factors_;
}

std::vector<float> orb_extractor::get_inv_scale_factors() const {
    return inv_scale_factors_;
}

std::vector<float> orb_extractor::get_level_sigma_sq() const {
    return level_sigma_sq_;
}

std::vector<float> orb_extractor::get_inv_level_sigma_sq() const {
    return inv_level_sigma_sq_;
}

void orb_extractor::create_point_pairs() {
    constexpr unsigned int num_sample_pts = 512;
    auto pattern = reinterpret_cast<const cv::Point*>(orb_point_pairs);
    std::copy(pattern, pattern + num_sample_pts, std::back_inserter(pattern_));
}

void orb_extractor::initialize() {
    // compute scale pyramid information
    calc_scale_factors();

    // resize buffers according to the number of levels
    image_pyramid_.resize(orb_params_.num_levels_);
    num_keypts_per_level_.resize(orb_params_.num_levels_);

    // compute the desired number of keypoints per scale
    double desired_num_keypts_per_scale
            = orb_params_.max_num_keypts_ * (1.0 - 1.0 / orb_params_.scale_factor_)
              / (1.0 - std::pow(1.0 / orb_params_.scale_factor_, static_cast<double>(orb_params_.num_levels_)));
    unsigned int total_num_keypts = 0;
    for (unsigned int level = 0; level < orb_params_.num_levels_ - 1; ++level) {
        num_keypts_per_level_.at(level) = std::round(desired_num_keypts_per_scale);
        total_num_keypts += num_keypts_per_level_.at(level);
        desired_num_keypts_per_scale *= 1.0 / orb_params_.scale_factor_;
    }
    num_keypts_per_level_.at(orb_params_.num_levels_ - 1) = std::max(static_cast<int>(orb_params_.max_num_keypts_) - static_cast<int>(total_num_keypts), 0);

    // orientation
    u_max_.resize(orb_params_.half_patch_size_ + 1);
    const unsigned int vmax = std::floor(orb_params_.half_patch_size_ * std::sqrt(2.0) / 2 + 1);
    const unsigned int vmin = std::ceil(orb_params_.half_patch_size_ * std::sqrt(2.0) / 2);
    for (unsigned int v = 0; v <= vmax; ++v) {
        u_max_.at(v) = std::round(std::sqrt(orb_params_.half_patch_size_ * orb_params_.half_patch_size_ - v * v));
    }
    for (unsigned int v = orb_params_.half_patch_size_, v0 = 0; vmin <= v; --v) {
        while (u_max_.at(v0) == u_max_.at(v0 + 1)) {
            ++v0;
        }
        u_max_.at(v) = v0;
        ++v0;
    }
}

void orb_extractor::calc_scale_factors() {
    scale_factors_ = orb_params::calc_scale_factors(orb_params_.num_levels_, orb_params_.scale_factor_);
    inv_scale_factors_ = orb_params::calc_inv_scale_factors(orb_params_.num_levels_, orb_params_.scale_factor_);
    level_sigma_sq_ = orb_params::calc_level_sigma_sq(orb_params_.num_levels_, orb_params_.scale_factor_);
    inv_level_sigma_sq_ = orb_params::calc_inv_level_sigma_sq(orb_params_.num_levels_, orb_params_.scale_factor_);
}

void orb_extractor::create_rectangle_mask(const unsigned int cols, const unsigned int rows) {
    if (rect_mask_.empty()) {
        rect_mask_ = cv::Mat(rows, cols, CV_8UC1, cv::Scalar(255));
    }
    // draw masks
    for (const auto& mask_rect : orb_params_.mask_rects_) {
        // draw black rectangle
        const unsigned int x_min = std::round(cols * mask_rect.at(0));
        const unsigned int x_max = std::round(cols * mask_rect.at(1));
        const unsigned int y_min = std::round(rows * mask_rect.at(2));
        const unsigned int y_max = std::round(rows * mask_rect.at(3));
        cv::rectangle(rect_mask_, cv::Point2i(x_min, y_min), cv::Point2i(x_max, y_max), cv::Scalar(0), -1, cv::LINE_AA);
    }
}

void orb_extractor::compute_image_pyramid(const cv::Mat& image) {
    for (unsigned int level = 0; level < orb_params_.num_levels_; ++level) {
        const double scale = scale_factors_.at(level);

        const cv::Size size(std::round(image.cols * 1.0 / scale), std::round(image.rows * 1.0 / scale));
        const cv::Size whole_size(size.width + orb_params_.edge_thr_ * 2, size.height + orb_params_.edge_thr_ * 2);

        const cv::Mat temp(whole_size, image.type());
        image_pyramid_.at(level) = temp(cv::Rect(orb_params_.edge_thr_, orb_params_.edge_thr_, size.width, size.height));

        if (level != 0) {
            cv::resize(image_pyramid_.at(level - 1), image_pyramid_.at(level), size, 0, 0, cv::INTER_LINEAR);
            cv::copyMakeBorder(image_pyramid_.at(level), temp,
                               orb_params_.edge_thr_, orb_params_.edge_thr_, orb_params_.edge_thr_, orb_params_.edge_thr_,
                               cv::BORDER_REFLECT_101 + cv::BORDER_ISOLATED);
        }
        else {
            cv::copyMakeBorder(image, temp,
                               orb_params_.edge_thr_, orb_params_.edge_thr_, orb_params_.edge_thr_, orb_params_.edge_thr_,
                               cv::BORDER_REFLECT_101);
        }
    }
}

void orb_extractor::compute_fast_keypoints(std::vector<std::vector<cv::KeyPoint>>& all_keypts, const cv::Mat& mask) const {
    all_keypts.resize(orb_params_.num_levels_);

    // mask(image or rectangle)のチェックをする関数
    auto is_in_mask = [&mask](const unsigned int y, const unsigned int x, const float scale_factor) {
        return mask.at<unsigned char>(y * scale_factor, x * scale_factor) == 0;
    };

    constexpr unsigned int cell_size = 64;

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int level = 0; level < orb_params_.num_levels_; ++level) {
        const float scale_factor = scale_factors_.at(level);

        const int min_border_x = orb_params_.edge_thr_ - 3;
        const int min_border_y = min_border_x;
        const unsigned int max_border_x = image_pyramid_.at(level).cols - orb_params_.edge_thr_ + 3;
        const unsigned int max_border_y = image_pyramid_.at(level).rows - orb_params_.edge_thr_ + 3;

        const unsigned int width = max_border_x - min_border_x;
        const unsigned int height = max_border_y - min_border_y;

        const unsigned int num_cols = width / cell_size;
        const unsigned int num_rows = height / cell_size;
        const unsigned int cell_width = std::ceil(width / num_cols);
        const unsigned int cell_height = std::ceil(height / num_rows);

        // 縦横cell_size[px]のグリッド（オーバーラップ6px）のそれぞれで仮の特徴点を計算して保存
        std::vector<cv::KeyPoint> keypts_to_distribute;
        keypts_to_distribute.reserve(orb_params_.max_num_keypts_ * 10);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (unsigned int i = 0; i < num_rows; ++i) {
            const unsigned int ini_y = min_border_y + i * cell_height;
            if (max_border_y - 3 <= ini_y) {
                continue;
            }
            unsigned int max_y = ini_y + cell_height + 6;
            if (max_border_y < max_y) {
                max_y = max_border_y;
            }

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
            for (unsigned int j = 0; j < num_cols; ++j) {
                const unsigned int ini_x = min_border_x + j * cell_width;
                if (max_border_x - 3 <= ini_x) {
                    continue;
                }
                unsigned int max_x = ini_x + cell_width + 6;
                if (max_border_x < max_x) {
                    max_x = max_border_x;
                }

                // patchの四隅いずれかがmask内である場合はFAST点を計算しない
                if (!mask.empty()) {
                    if (is_in_mask(ini_y, ini_x, scale_factor) || is_in_mask(max_y, ini_x, scale_factor)
                        || is_in_mask(ini_y, max_x, scale_factor) || is_in_mask(max_y, max_x, scale_factor)) {
                        continue;
                    }
                }

                std::vector<cv::KeyPoint> keypts_in_cell;
                cv::FAST(image_pyramid_.at(level).rowRange(ini_y, max_y).colRange(ini_x, max_x),
                         keypts_in_cell, orb_params_.ini_fast_thr_, true);

                // 特徴点がなければ閾値を下げて再計算
                if (keypts_in_cell.empty()) {
                    cv::FAST(image_pyramid_.at(level).rowRange(ini_y, max_y).colRange(ini_x, max_x),
                             keypts_in_cell, orb_params_.min_fast_thr, true);
                }

                if (keypts_in_cell.empty()) {
                    continue;
                }

                // 特徴点をスケールごとに集める
#ifdef USE_OPENMP
#pragma omp critical
#endif
                {
                    for (auto& keypt : keypts_in_cell) {
                        keypt.pt.x += j * cell_width;
                        keypt.pt.y += i * cell_height;
                        // mask内かどうかを確認
                        if (!mask.empty() && is_in_mask(keypt.pt.y + min_border_y, keypt.pt.x + min_border_x, scale_factor)) {
                            continue;
                        }
                        keypts_to_distribute.push_back(keypt);
                    }
                }
            }
        }

        std::vector<cv::KeyPoint>& keypts_at_level = all_keypts.at(level);
        keypts_at_level.reserve(orb_params_.max_num_keypts_);

        // スケールごとの特徴点を木構造を用いて選別
        keypts_at_level = distribute_keypoints_via_tree(keypts_to_distribute,
                                                        min_border_x, max_border_x, min_border_y, max_border_y,
                                                        num_keypts_per_level_.at(level));

        // patch sizeをscaleで補正したものがkeypoint sizeになる
        const unsigned int scaled_patch_size = orb_params_.patch_size_ * scale_factors_.at(level);

        for (auto& keypt : keypts_at_level) {
            // 座標は平行移動のみ補正(スケールについてはORB記述の後まで補正しない)
            keypt.pt.x += min_border_x;
            keypt.pt.y += min_border_y;
            // 座標以外の情報をセットする
            keypt.octave = level;
            keypt.size = scaled_patch_size;
        }
    }

    // compute orientations
    for (unsigned int level = 0; level < orb_params_.num_levels_; ++level) {
        compute_orientation(image_pyramid_.at(level), all_keypts.at(level));
    }
}

std::vector<cv::KeyPoint> orb_extractor::distribute_keypoints_via_tree(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                                       const int min_x, const int max_x, const int min_y, const int max_y,
                                                                       const unsigned int num_keypts) const {
    auto nodes = initialize_nodes(keypts_to_distribute, min_x, max_x, min_y, max_y);

    // これ以上の分岐が可能な leaf node のベクトル
    // 次イテレーションの分岐で指定特徴点数を超えるときに使用される
    // 指定の特徴点数で切り上げるときにkeypoint数でソートされる
    std::vector<std::pair<int, orb_extractor_node*>> leaf_nodes_pool;
    leaf_nodes_pool.reserve(nodes.size() * 10);

    // 特徴点(keypts_)が指定数確保されているかを表すフラグ
    bool is_filled = false;

    while (true) {
        const unsigned int prev_size = nodes.size();

        auto iter = nodes.begin();
        leaf_nodes_pool.clear();

        // nodeを分岐し，分岐前のnodeをnodesから消去する
        while (iter != nodes.end()) {
            if (iter->is_leaf_node_) {
                iter++;
                continue;
            }

            // nodeを分岐して割り当てる
            const auto child_nodes = iter->divide_node();
            assign_child_nodes(child_nodes, nodes, leaf_nodes_pool);
            // 分割前の不要なnodeを消去
            iter = nodes.erase(iter);
        }

        // node数が指定特徴点数を超えるか，または木の分岐が進まなくなったとき分岐を終了する
        if (num_keypts <= nodes.size() || nodes.size() == prev_size) {
            is_filled = true;
            break;
        }

        // 次イテレーションでの特徴点数が指定数を超える場合は，
        // 最終ノードを追加してキープする特徴点を決定する
        if (num_keypts < nodes.size() + leaf_nodes_pool.size()) {
            is_filled = false;
            break;
        }
    }

    while (!is_filled) {
        const unsigned int prev_size = nodes.size();

        auto prev_leaf_nodes_pool = leaf_nodes_pool;
        leaf_nodes_pool.clear();

        // leaf nodeが持つ特徴点数で降順ソート
        std::sort(prev_leaf_nodes_pool.rbegin(), prev_leaf_nodes_pool.rend());
        // 持っている特徴点が多いleaf nodeから処理
        for (const auto& prev_leaf_node : prev_leaf_nodes_pool) {
            // nodeを分岐して割り当てる
            const auto child_nodes = prev_leaf_node.second->divide_node();
            assign_child_nodes(child_nodes, nodes, leaf_nodes_pool);
            // 分割前の不要なnodeを消去
            nodes.erase(prev_leaf_node.second->iter_);

            if (num_keypts <= nodes.size()) {
                is_filled = true;
                break;
            }
        }

        // leaf node数が指定数に到達したか分岐が進まなくなったとき分岐を終了する
        if (is_filled || num_keypts <= nodes.size() || nodes.size() == prev_size) {
            is_filled = true;
            break;
        }
    }

    return find_keypoints_with_max_response(nodes);
}

std::list<orb_extractor_node> orb_extractor::initialize_nodes(const std::vector<cv::KeyPoint>& keypts_to_distribute,
                                                              const int min_x, const int max_x, const int min_y, const int max_y) const {
    // rootから分岐するnodeの数
    // 各nodeに割り当てられる画像領域がおおよそ正方形になるように割り当てる
    const unsigned int num_initial_nodes = std::round(static_cast<double>(max_x - min_x) / (max_y - min_y));
    // rootから分岐するnodeのx方向の画像領域幅
    const auto delta_x = static_cast<double>(max_x - min_x) / num_initial_nodes;

    // nodeのリスト
    // leaf nodeまたは暫定的に分岐可能なnodeを保持する．木構造は保持していない
    std::list<orb_extractor_node> nodes;

    // rootから分岐したnode
    std::vector<orb_extractor_node*> initial_nodes;
    initial_nodes.resize(num_initial_nodes);

    // initial_nodes を初期化し，nodesに格納する
    for (unsigned int i = 0; i < num_initial_nodes; ++i) {
        orb_extractor_node node;

        node.pt_begin_ = cv::Point2i(delta_x * static_cast<double>(i), 0);
        node.pt_end_ = cv::Point2i(delta_x * static_cast<double>(i + 1), max_y - min_y);
        node.keypts_.reserve(keypts_to_distribute.size());

        nodes.push_back(node);
        initial_nodes.at(i) = &nodes.back();
    }

    // 与えられた特徴点をinitial_nodesに格納する．
    for (const auto& keypt : keypts_to_distribute) {
        initial_nodes.at(keypt.pt.x / delta_x)->keypts_.push_back(keypt);
    }

    // leaf nodeの判定と特徴点が空なleaf nodeの消去を行う
    auto iter = nodes.begin();
    while (iter != nodes.end()) {
        // 空だったら消去する
        if (iter->keypts_.empty()) {
            iter = nodes.erase(iter);
            continue;
        }
        iter->is_leaf_node_ = (iter->keypts_.size() == 1);
        iter++;
    }

    return nodes;
}

void orb_extractor::assign_child_nodes(const std::array<orb_extractor_node, 4>& child_nodes, std::list<orb_extractor_node>& nodes,
                                       std::vector<std::pair<int, orb_extractor_node*>>& leaf_nodes) const {
    for (const auto& child_node : child_nodes) {
        if (child_node.keypts_.empty()) {
            continue;
        }
        nodes.push_front(child_node);
        if (child_node.keypts_.size() == 1) {
            continue;
        }
        leaf_nodes.emplace_back(std::make_pair(child_node.keypts_.size(), &nodes.front()));
        // nodes上の自身のiterを持っておく
        nodes.front().iter_ = nodes.begin();
    }
}

std::vector<cv::KeyPoint> orb_extractor::find_keypoints_with_max_response(std::list<orb_extractor_node>& nodes) const {
    // 出力される特徴点のvector
    std::vector<cv::KeyPoint> result_keypts;
    result_keypts.reserve(nodes.size());

    // 各nodeの画像領域内の特徴点の中でresponseが最大となる特徴点をresult_keyptsに格納する
    for (auto& node : nodes) {
        auto& node_keypts = node.keypts_;
        auto& keypt = node_keypts.at(0);
        double max_response = keypt.response;

        for (unsigned int k = 1; k < node_keypts.size(); ++k) {
            if (node_keypts.at(k).response > max_response) {
                keypt = node_keypts.at(k);
                max_response = node_keypts.at(k).response;
            }
        }

        result_keypts.push_back(keypt);
    }

    return result_keypts;
}

void orb_extractor::compute_orientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypts) const {
    for (auto& keypt : keypts) {
        keypt.angle = ic_angle(image, keypt.pt);
    }
}

void orb_extractor::correct_keypoint_scale(std::vector<cv::KeyPoint>& keypts_at_level, const unsigned int level) const {
    if (level == 0) {
        return;
    }
    const float scale_at_level = scale_factors_.at(level);
    for (auto& keypt_at_level : keypts_at_level) {
        keypt_at_level.pt *= scale_at_level;
    }
}

float orb_extractor::ic_angle(const cv::Mat& image, const cv::Point2f& point) const {
    int m_01 = 0, m_10 = 0;

    const uchar* const center = &image.at<uchar>(cvRound(point.y), cvRound(point.x));

    for (int u = -orb_params_.half_patch_size_; u <= orb_params_.half_patch_size_; ++u) {
        m_10 += u * center[u];
    }

    const auto step = static_cast<int>(image.step1());
    for (int v = 1; v <= orb_params_.half_patch_size_; ++v) {
        unsigned int v_sum = 0;
        const int d = u_max_.at(v);
        for (int u = -d; u <= d; ++u) {
            const int val_plus = center[u + v * step];
            const int val_minus = center[u - v * step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return cv::fastAtan2(m_01, m_10);
}

void orb_extractor::compute_orb_descriptors(const cv::Mat& image, const std::vector<cv::KeyPoint>& keypts, cv::Mat& descriptors) const {
    descriptors = cv::Mat::zeros(keypts.size(), 32, CV_8UC1);

    for (unsigned int i = 0; i < keypts.size(); ++i) {
        compute_orb_descriptor(keypts.at(i), image, &pattern_.at(0), descriptors.ptr(i));
    }
}

void orb_extractor::compute_orb_descriptor(const cv::KeyPoint& keypt, const cv::Mat& image, const cv::Point* pattern, uchar* desc) const {
    const float angle = keypt.angle * M_PI / 180.0;
    const float cos_angle = util::cos(angle);
    const float sin_angle = util::sin(angle);

    const uchar* const center = &image.at<uchar>(cvRound(keypt.pt.y), cvRound(keypt.pt.x));
    const auto step = static_cast<int>(image.step);

#define GET_VALUE(idx) \
        center[cvRound(pattern[idx].x * sin_angle + pattern[idx].y * cos_angle) * step \
               + cvRound(pattern[idx].x * cos_angle - pattern[idx].y * sin_angle)]

    for (unsigned int i = 0; i < 32; ++i, pattern += 16) {
        int t0, t1, val;

        t0 = GET_VALUE(0);
        t1 = GET_VALUE(1);
        val = t0 < t1;
        t0 = GET_VALUE(2);
        t1 = GET_VALUE(3);
        val |= (t0 < t1) << 1;
        t0 = GET_VALUE(4);
        t1 = GET_VALUE(5);
        val |= (t0 < t1) << 2;
        t0 = GET_VALUE(6);
        t1 = GET_VALUE(7);
        val |= (t0 < t1) << 3;
        t0 = GET_VALUE(8);
        t1 = GET_VALUE(9);
        val |= (t0 < t1) << 4;
        t0 = GET_VALUE(10);
        t1 = GET_VALUE(11);
        val |= (t0 < t1) << 5;
        t0 = GET_VALUE(12);
        t1 = GET_VALUE(13);
        val |= (t0 < t1) << 6;
        t0 = GET_VALUE(14);
        t1 = GET_VALUE(15);
        val |= (t0 < t1) << 7;

        desc[i] = static_cast<uchar>(val);
    }

#undef GET_VALUE

}

} // namespace feature
} // namespace openvslam
