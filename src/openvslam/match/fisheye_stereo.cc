#include "openvslam/match/fisheye_stereo.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace openvslam {
namespace match {

fisheye_stereo::fisheye_stereo(const camera::fisheye* camera,
        const std::vector<cv::Mat>& left_image_pyramid, const std::vector<cv::Mat>& right_image_pyramid,
                               const std::vector<cv::KeyPoint>& undistort_keypts_left, const std::vector<cv::KeyPoint>& undistort_keypts_right,
                               const cv::Mat& descs_left, const cv::Mat& descs_right,
                               const std::vector<float>& scale_factors, const std::vector<float>& inv_scale_factors
                               )
    : camera_(camera),left_image_pyramid_(left_image_pyramid), right_image_pyramid_(right_image_pyramid),
      num_keypts_(undistort_keypts_left.size()),
      undistort_keypts_left_(undistort_keypts_left), undistort_keypts_right_(undistort_keypts_right),
      descs_left_(descs_left), descs_right_(descs_right),
      scale_factors_(scale_factors), inv_scale_factors_(inv_scale_factors),
      focal_x_baseline_(camera_->focal_x_baseline_), true_baseline_(camera_->true_baseline_),
      min_disp_(0.0f), max_disp_(focal_x_baseline_ / true_baseline_){}

void fisheye_stereo::compute(std::vector<float>& stereo_x_right, std::vector<float>& depths) const {
    // 画像の行ごとに，ORBで抽出した右画像の特徴点indexを格納しておく
//    std::chrono::system_clock::time_point  start, end;
//    start = std::chrono::system_clock::now();
    const auto indices_right_in_row = get_right_keypoint_indices_in_each_row(2.0);
//    end = std::chrono::system_clock::now();
//    double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
//    std::cout << "elapsed:" << elapsed/1000  << "[ms]:" << std::endl;

    // 左画像の各特徴点についてサブピクセルで視差と深度を求める
    const int num_y_max = camera_->img_bounds_.max_y_;
    const int num_y_min = camera_->img_bounds_.min_y_;
    stereo_x_right.resize(num_keypts_, -1.0f);
    depths.resize(num_keypts_, -1.0f);
    std::vector<std::pair<int, int>> correlation_and_idx_left;
    correlation_and_idx_left.reserve(num_keypts_);

#ifdef USE_OPENMP
#pragma omp parallel for
#endif
    for (unsigned int idx_left = 0; idx_left < num_keypts_; ++idx_left) {
        const auto& keypt_left = undistort_keypts_left_.at(idx_left);
        const auto scale_level_left = keypt_left.octave;
        const float y_left = keypt_left.pt.y;
        const float x_left = keypt_left.pt.x;

        // 左画像の特徴点と同じ高さにある右画像の特徴点のindexを取得 -> マッチング候補
        if( y_left <  num_y_min+1 ||  num_y_max-1 < y_left) continue;
        const auto& candidate_indices_right = indices_right_in_row.at(y_left - num_y_min);
        if (candidate_indices_right.empty()) {
            continue;
        }

        // 右画像のxの許容範囲を計算する
        const float min_x_right = x_left - max_disp_;
        const float max_x_right = x_left - min_disp_;
        if (max_x_right < 0) {
            continue;
        }

        // 左画像のidx_leftと特徴ベクトルが一番近いbest_idx_rightを探す
        unsigned int best_idx_right = 0;
        unsigned int best_hamm_dist = hamm_dist_thr_;
        find_closest_keypoints_in_stereo(idx_left, scale_level_left, candidate_indices_right,
                                         min_x_right, max_x_right, best_idx_right, best_hamm_dist);
        // ハミング距離が閾値を満たさなければ破棄
        if (hamm_dist_thr_ <= best_hamm_dist) {
            continue;
        }
        const auto& keypt_right = undistort_keypts_right_.at(best_idx_right);
#define SUBPIXEL
#ifndef SUBPIXEL
        // パッチの相関を求めて，サブピクセルオーダーの視差を求める
        float best_x_right = keypt_right.pt.x;
        float best_disp = keypt_left.pt.x - best_x_right;
#endif

#ifdef SUBPIXEL
        float best_x_right = -1.0f;
        float best_disp = -1.0f;
        float best_correlation = UINT_MAX;
        const auto is_valid = compute_subpixel_disparity(keypt_left, keypt_right, best_x_right, best_disp, best_correlation);
        // 見つからなければ破棄
        if (!is_valid) {
            continue;
        }
        // 視差が有効範囲外であれば破棄
        if (best_disp < min_disp_ || max_disp_ <= best_disp) {
            continue;
        }

        // 視差が範囲内であれば情報を保存する
        if (best_disp <= 0.0f) {
            // 視差が0の場合は微小量を設定しておく
            best_disp = 0.01f;
            best_x_right = x_left - best_disp;
        }
//        std::cout << keypt_left.pt.x <<"best disp before: " << keypt_left.pt.x - keypt_right.pt.x << ", after:" << best_disp << std::endl;

#endif
        // 計算結果をセット
        // depths, stereo_x_right についてはループごとに別のメモリ領域にアクセスするのでcritical指定は必要ない
        depths.at(idx_left) = focal_x_baseline_ / best_disp;
        stereo_x_right.at(idx_left) = best_x_right;
#ifdef SUBPIXEL
        // こっちはcritical指定が必要
#ifdef USE_OPENMP
#pragma omp critical
#endif
        {
            correlation_and_idx_left.emplace_back(std::make_pair(best_correlation, idx_left));
        }
#endif
    }
#ifdef SUBPIXEL
    // 相関の中央値を求める
    std::sort(correlation_and_idx_left.begin(), correlation_and_idx_left.end());
    const auto median_i = correlation_and_idx_left.size() / 2;
    const float median_correlation = correlation_and_idx_left.empty()
                                         ? 0.0f
                                         : correlation_and_idx_left.at(median_i).first;
    // 相関の中央値x2より相関が弱いものは破棄する
    const float correlation_thr = 2.0 * median_correlation;

    // 相関の中央値x2を閾値としているので，iはmedian_iから始めればよい
    for (unsigned int i = median_i; i < correlation_and_idx_left.size(); ++i) {
        const auto correlation = correlation_and_idx_left.at(i).first;
        const auto idx_left = correlation_and_idx_left.at(i).second;
        if (correlation_thr < correlation) {
            stereo_x_right.at(idx_left) = -1;
            depths.at(idx_left) = -1;
        }
    }
#endif
}

std::vector<std::vector<unsigned int>> fisheye_stereo::get_right_keypoint_indices_in_each_row(const float margin) const {
    // 画像の行ごとに，ORBで抽出した右画像の特徴点indexを格納しておく
    const int num_y_max = camera_->img_bounds_.max_y_;
    const int num_y_min = camera_->img_bounds_.min_y_;
    const unsigned int num_img_rows = num_y_max - num_y_min;

    std::vector<std::vector<unsigned int>> indices_right_in_row(num_img_rows, std::vector<unsigned int>());
    for (unsigned int row = 0; row < num_img_rows; ++row) {
        indices_right_in_row.at(row).reserve(100);
    }

    const unsigned int num_keypts_right = undistort_keypts_right_.size();
    for (unsigned int idx_right = 0; idx_right < num_keypts_right; ++idx_right) {
        // 右画像の特徴点のy座標を取得
        const auto& keypt_right = undistort_keypts_right_.at(idx_right);
        const float y_right = keypt_right.pt.y;
        // スケールに応じて座標の不定性を設定
        const float r = margin * scale_factors_.at(undistort_keypts_right_.at(idx_right).octave);
        // 上端と下端を計算
        const int max_r = cvCeil(y_right + r);
        const int min_r = cvFloor(y_right - r);

        // 上端と下端の間の行番号すべてについて，特徴点indexを保存しておく
        for (int row_right = min_r; row_right <= max_r; ++row_right) {
            if( row_right <  num_y_min+1 ||  num_y_max-1 < row_right) continue;
            indices_right_in_row.at(row_right - num_y_min).push_back(idx_right);
        }
    }
    return indices_right_in_row;
}

void fisheye_stereo::find_closest_keypoints_in_stereo(const unsigned int idx_left, const int scale_level_left,
                                              const std::vector<unsigned int>& candidate_indices_right,
                                              const float min_x_right, const float max_x_right,
                                              unsigned int& best_idx_right, unsigned int& best_hamm_dist) const {
    best_idx_right = 0;
    best_hamm_dist = hamm_dist_thr_;
    unsigned int second_hamm_dist = hamm_dist_thr_;
    const cv::Mat& desc_left = descs_left_.row(idx_left);
    // 左画像の特徴点と右画像の各特徴点のハミング距離を計算する
    // 左画像の特徴点に対して，一番近い右画像の特徴点indexを取得する -> best_idx_right
    for (const auto idx_right : candidate_indices_right) {
        const auto& keypt_right = undistort_keypts_right_.at(idx_right);
        // ORBスケールが大きく異なる場合は破棄
        if (keypt_right.octave < scale_level_left - 1 || keypt_right.octave > scale_level_left + 1) {
            continue;
        }

        // 視差をチェックして許容範囲外だったら破棄
        const float x_right = keypt_right.pt.x;
        if (x_right < min_x_right || max_x_right < x_right) {
            continue;
        }

        // ハミング距離を計算
        const auto& desc_right = descs_right_.row(idx_right);
        const unsigned int hamm_dist = match::compute_descriptor_distance_32(desc_left, desc_right);

        if (hamm_dist < best_hamm_dist) {
            best_idx_right = idx_right;
            second_hamm_dist = best_hamm_dist;
            best_hamm_dist = hamm_dist;
        }
    }

    if(second_hamm_dist != hamm_dist_thr_ && best_hamm_dist > second_hamm_dist*0.6){
        best_idx_right = 0;
        best_hamm_dist = hamm_dist_thr_;
    }
}
bool fisheye_stereo::compute_subpixel_disparity(const cv::KeyPoint& keypt_left, const cv::KeyPoint& keypt_right,
                                        float& best_x_right, float& best_disp, float& best_correlation) const {
    // 最もハミング距離が近い右画像の特徴点
    const float x_right = keypt_right.pt.x;

    // パッチの移動範囲を計算し，範囲外であれば破棄
    constexpr int win_size = 5;
    constexpr int slide_width = 5;
    const int ini_x = keypt_right.pt.x + slide_width - win_size;
    const int end_x = keypt_right.pt.x + slide_width + win_size + 1;

    if (ini_x < camera_->img_bounds_.min_x_ || camera_->img_bounds_.max_x_ < end_x) {
        return false;
    }

    // 特徴点周辺画素の相関を計算して，放物線フィッティングによりサブピクセルオーダーの視差を求める
    best_correlation = UINT_MAX;
    int best_offset = 0;
    std::vector<float> correlations(2 * slide_width + 1, -1);

    // 左画像のパッチを抽出する
    cv::Mat patch_left;
    rectify_patch_image(keypt_left, image_side::Left, cv::Size(win_size,win_size), patch_left);
//    cv::imwrite("left_patch.png", patch_left);

    patch_left.convertTo(patch_left, CV_32F);
    patch_left -= patch_left.at<float>(win_size, win_size) * cv::Mat::ones(patch_left.rows, patch_left.cols, CV_32F);

    // 右画像のパッチを抽出する (slide_width込)
    cv::Mat patch_right_wide;
    rectify_patch_image(keypt_right, image_side::Right, cv::Size(win_size+slide_width,win_size), patch_right_wide);
//    cv::imwrite("patch_right_wide.png", patch_right_wide);
    for (int offset = -slide_width; offset <= +slide_width; ++offset) {
        auto patch_right = patch_right_wide.colRange(offset+slide_width, offset+slide_width+2*win_size+1);
        patch_right.convertTo(patch_right, CV_32F);
        patch_right -= patch_right.at<float>(win_size, win_size) * cv::Mat::ones(patch_right.rows, patch_right.cols, CV_32F);

        // L1相関を求める
        const float correlation = cv::norm(patch_left, patch_right, cv::NORM_L1);
        if (correlation < best_correlation) {
            best_correlation = correlation;
            best_offset = offset;
        }

        correlations.at(slide_width + offset) = correlation;
    }

    if (best_offset == -slide_width || best_offset == slide_width) {
        return false;
    }

    // 一番相関が強い点を中心に，3点の相関値を取り出して放物線をフィッティングする
    const float correlation_1 = correlations.at(slide_width + best_offset - 1);
    const float correlation_2 = correlations.at(slide_width + best_offset);
    const float correlation_3 = correlations.at(slide_width + best_offset + 1);

    // best_offsetからの平行移動量を計算する
    float x_delta;
    constexpr bool is_parabolic_fitting = false;
    if(is_parabolic_fitting){
        // x_delta : (-1, correlation_1),(0, correlation_2),(+1, correlation_1)の3点を通る放物線の頂点座標
        x_delta = (correlation_1 - correlation_3) / (2.0 * (correlation_1 + correlation_3) - 4.0 * correlation_2);
    }else{
        // linear fitting
        if(correlation_1>correlation_3){
            x_delta =(correlation_3-correlation_1)/(2*(correlation_2-correlation_1));
        }else{
            x_delta =(correlation_3-correlation_1)/(2*(correlation_2-correlation_3));
        }
    }

    if (x_delta < -1.0 || 1.0 < x_delta) {
        return false;
    }

    // 元の画像での視差を計算する
    best_x_right = x_right + best_offset + x_delta;
    best_disp = keypt_left.pt.x - best_x_right;

    return true;
}


void fisheye_stereo::rectify_patch_image(const cv::KeyPoint& keypt, const image_side& img_side, const cv::Size& half_patch_size, cv::Mat &patch) const{
    // patch grid
    const cv::Size patch_size = half_patch_size*2 + cv::Size(1,1);
    cv::Mat grid_x = cv::Mat::zeros(patch_size, CV_32FC1);
    cv::Mat grid_y = cv::Mat::zeros(patch_size, CV_32FC1);
    cv::Mat grid_z = cv::Mat::ones(patch_size, CV_32FC1);
    for(int j=-half_patch_size.height; j < half_patch_size.height + 1; ++j){
        for(int i=-half_patch_size.width; i < half_patch_size.width + 1; ++i){
            int i_idx = i + half_patch_size.width;
            int j_idx = j + half_patch_size.height;
            grid_x.at<float>(j_idx, i_idx) = i +keypt.pt.x;
            grid_y.at<float>(j_idx, i_idx) = j +keypt.pt.y;
        }
    }

    // transform points
    cv::Mat K, Kleft, D, rvec;
    cv::Mat grid;
    cv::Mat points;
    const cv::Mat src = img_side == image_side::Left ? left_image_pyramid_.at(0) : right_image_pyramid_.at(0);
    if(img_side == image_side::Left){
        Kleft = camera_->cv_cam_matrix_;
        K = camera_->cv_cam_matrix_;
        D = camera_->cv_dist_params_;
        cv::Rodrigues(camera_->R1_.t(), rvec);
    }else if(img_side == image_side::Right){
        Kleft = camera_->cv_cam_matrix_;
        K = camera_->cv_cam_matrix_right_;
        D = camera_->cv_dist_params_right_;
        cv::Rodrigues(camera_->R2_.t(), rvec);
    }
    grid_x = (grid_x - Kleft.at<float>(0, 2)) / Kleft.at<float>(0, 0);
    grid_y = (grid_y - Kleft.at<float>(1, 2)) / Kleft.at<float>(1, 1);
    grid_x = grid_x.reshape(1, 1);
    grid_y = grid_y.reshape(1, 1);
    grid_z = grid_z.reshape(1, 1);
    cv::merge(std::vector<cv::Mat>{grid_x, grid_y, grid_z}, grid);

    cv::fisheye::projectPoints(grid, points, rvec, cv::Mat::zeros(3, 1, CV_32FC1), K, D);
    points = points.reshape(0, patch_size.height);

    // remap image
    cv::remap(src, patch, points, cv::Mat(), cv::INTER_NEAREST);
}

} // namespace match
} // namespace openvslam
