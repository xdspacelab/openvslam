#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/publish/frame_publisher.h"

#include <spdlog/spdlog.h>
#include <opencv2/imgproc.hpp>

namespace openvslam {
namespace publish {

frame_publisher::frame_publisher(const std::shared_ptr<config>& cfg, data::map_database* map_db,
                                 const unsigned int img_width)
    : cfg_(cfg), map_db_(map_db), img_width_(img_width),
      img_(cv::Mat(480, img_width_, CV_8UC3, cv::Scalar(0, 0, 0))) {
    spdlog::debug("CONSTRUCT: publish::frame_publisher");
}

frame_publisher::~frame_publisher() {
    spdlog::debug("DESTRUCT: publish::frame_publisher");
}

cv::Mat frame_publisher::draw_frame(const bool draw_text) {
    cv::Mat img;
    tracker_state_t tracking_state;
    std::vector<cv::KeyPoint> init_keypts;
    std::vector<int> init_matches;
    std::vector<cv::KeyPoint> curr_keypts;
    double elapsed_ms;
    bool mapping_is_enabled;
    std::vector<bool> is_tracked;

    // copy to avoid memory access conflict
    {
        std::lock_guard<std::mutex> lock(mtx_);

        img_.copyTo(img);

        tracking_state = tracking_state_;

        // copy tracking information
        if (tracking_state == tracker_state_t::Initializing) {
            init_keypts = init_keypts_;
            init_matches = init_matches_;
        }

        curr_keypts = curr_keypts_;

        elapsed_ms = elapsed_ms_;

        mapping_is_enabled = mapping_is_enabled_;

        is_tracked = is_tracked_;
    }

    // resize image
    const float mag = (img_width_ < img_.cols) ? static_cast<float>(img_width_) / img.cols : 1.0;
    if (mag != 1.0) {
        cv::resize(img, img, cv::Size(), mag, mag, cv::INTER_NEAREST);
    }

    // to draw COLOR information
    if (img.channels() < 3) {
        cvtColor(img, img, cv::COLOR_GRAY2BGR);
    }

    // draw keypoints
    unsigned int num_tracked = 0;
    switch (tracking_state) {
        case tracker_state_t::Initializing: {
            num_tracked = draw_initial_points(img, init_keypts, init_matches, curr_keypts, mag);
            break;
        }
        case tracker_state_t::Tracking: {
            num_tracked = draw_tracked_points(img, curr_keypts, is_tracked, mapping_is_enabled, mag);
            break;
        }
        default: {
            break;
        }
    }

    if (draw_text) {
        // draw tracking info
        draw_info_text(img, tracking_state, num_tracked, elapsed_ms, mapping_is_enabled);
    }

    return img;
}

unsigned int frame_publisher::draw_initial_points(cv::Mat& img, const std::vector<cv::KeyPoint>& init_keypts,
                                                  const std::vector<int>& init_matches, const std::vector<cv::KeyPoint>& curr_keypts,
                                                  const float mag) const {
    unsigned int num_tracked = 0;

    for (unsigned int i = 0; i < init_matches.size(); ++i) {
        if (init_matches.at(i) < 0) {
            continue;
        }

        cv::circle(img, init_keypts.at(i).pt * mag, 2, mapping_color_, -1);
        cv::circle(img, curr_keypts.at(init_matches.at(i)).pt * mag, 2, mapping_color_, -1);
        cv::line(img, init_keypts.at(i).pt * mag, curr_keypts.at(init_matches.at(i)).pt * mag, mapping_color_);

        ++num_tracked;
    }

    return num_tracked;
}

unsigned int frame_publisher::draw_tracked_points(cv::Mat& img, const std::vector<cv::KeyPoint>& curr_keypts,
                                                  const std::vector<bool>& is_tracked, const bool mapping_is_enabled,
                                                  const float mag) const {
    constexpr float radius = 5;

    unsigned int num_tracked = 0;

    for (unsigned int i = 0; i < curr_keypts.size(); ++i) {
        if (!is_tracked.at(i)) {
            continue;
        }

        const cv::Point2f pt_begin{curr_keypts.at(i).pt.x * mag - radius, curr_keypts.at(i).pt.y * mag - radius};
        const cv::Point2f pt_end{curr_keypts.at(i).pt.x * mag + radius, curr_keypts.at(i).pt.y * mag + radius};

        if (mapping_is_enabled) {
            cv::rectangle(img, pt_begin, pt_end, mapping_color_);
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, mapping_color_, -1);
        }
        else {
            cv::rectangle(img, pt_begin, pt_end, localization_color_);
            cv::circle(img, curr_keypts.at(i).pt * mag, 2, localization_color_, -1);
        }

        ++num_tracked;
    }

    return num_tracked;
}

void frame_publisher::draw_info_text(cv::Mat& img, const tracker_state_t tracking_state, const unsigned int num_tracked,
                                     const double elapsed_ms, const bool mapping_is_enabled) const {
    // create text info
    std::stringstream ss;
    switch (tracking_state) {
        case tracker_state_t::NotInitialized: {
            ss << "WAITING FOR IMAGES ";
            break;
        }
        case tracker_state_t::Initializing: {
            ss << "INITIALIZE | ";
            ss << "KP: " << num_tracked << ", ";
            ss << "track time: " << std::fixed << std::setprecision(0) << elapsed_ms << "ms";
            break;
        }
        case tracker_state_t::Tracking: {
            ss << (mapping_is_enabled ? "MAPPING | " : "LOCALIZATION | ");
            ss << "KF: " << map_db_->get_num_keyframes() << ", ";
            ss << "LM: " << map_db_->get_num_landmarks() << ", ";
            ss << "KP: " << num_tracked << ", ";
            ss << "track time: " << std::fixed << std::setprecision(0) << elapsed_ms << "ms";
            break;
        }
        case tracker_state_t::Lost: {
            ss << "LOST | ";
            ss << "track time: " << std::fixed << std::setprecision(0) << elapsed_ms << "ms";
            break;
        }
    }

    int baseline = 0;
    const cv::Size text_size = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

    // create text area
    constexpr float alpha = 0.6;
    cv::Mat overlay(img.rows, img.cols, img.type());
    img.copyTo(overlay);
    cv::rectangle(overlay, cv::Point2i{0, img.rows - text_size.height - 10}, cv::Point2i{img.cols, img.rows}, cv::Scalar{0, 0, 0}, -1);
    cv::addWeighted(overlay, alpha, img, 1 - alpha, 0, img);
    // put text
    cv::putText(img, ss.str(), cv::Point(5, img.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar{255, 255, 255}, 1, 8);
}

void frame_publisher::update(tracking_module* tracker) {
    std::lock_guard<std::mutex> lock(mtx_);

    tracker->img_gray_.copyTo(img_);

    const auto num_curr_keypts = tracker->curr_frm_.num_keypts_;
    curr_keypts_ = tracker->curr_frm_.keypts_;
    elapsed_ms_ = tracker->elapsed_ms_;
    mapping_is_enabled_ = tracker->get_mapping_module_status();
    tracking_state_ = tracker->last_tracking_state_;

    is_tracked_ = std::vector<bool>(num_curr_keypts, false);

    switch (tracking_state_) {
        case tracker_state_t::Initializing: {
            init_keypts_ = tracker->get_initial_keypoints();
            init_matches_ = tracker->get_initial_matches();
            break;
        }
        case tracker_state_t::Tracking: {
            for (unsigned int i = 0; i < num_curr_keypts; ++i) {
                const auto& lm = tracker->curr_frm_.landmarks_.at(i);
                if (!lm) {
                    continue;
                }
                if (tracker->curr_frm_.outlier_flags_.at(i)) {
                    continue;
                }

                if (0 < lm->num_observations()) {
                    is_tracked_.at(i) = true;
                }
            }
            break;
        }
        default: {
            break;
        }
    }
}

} // namespace publish
} // namespace openvslam
