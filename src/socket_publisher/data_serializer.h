#ifndef SOCKET_PUBLISHER_DATA_SERIALIZER_H
#define SOCKET_PUBLISHER_DATA_SERIALIZER_H

#include "openvslam/type.h"

#include <memory>

#include <Eigen/Core>
#include <sio_client.h>
#include <opencv2/opencv.hpp>

namespace openvslam {

class config;

namespace data {
class keyframe;
class landmark;
} // namespace data

namespace publisher {
class frame_publisher;
class map_publisher;
} // namespace publisher

} // namespace openvslam

namespace socket_publisher {

class data_serializer {
public:
    data_serializer(const std::shared_ptr<openvslam::publisher::frame_publisher>& frame_publisher,
                    const std::shared_ptr<openvslam::publisher::map_publisher>& map_publisher,
                    const unsigned int image_width, const unsigned int image_height);

    std::string serialize_messages(const std::vector<std::string>& tags, const std::vector<std::string>& messages);

    std::string serialize_map_diff();

    std::string serialize_latest_frame(const unsigned int image_quality_);

    static std::string serialized_reset_signal_;

private:
    const std::shared_ptr<openvslam::publisher::frame_publisher> frame_publisher_;
    const std::shared_ptr<openvslam::publisher::map_publisher> map_publisher_;
    const unsigned int image_width_;
    const unsigned int image_height_;
    std::unique_ptr<std::unordered_map<unsigned int, double>> keyframe_hash_map_;
    std::unique_ptr<std::unordered_map<unsigned int, double>> point_hash_map_;

    double current_pose_hash_ = 0;
    int frame_hash_ = 0;

    inline double get_vec_hash(const openvslam::Vec3_t& point) {
        return point[0] + point[1] + point[2];
    }

    inline double get_mat_hash(const openvslam::Mat44_t& pose) {
        return pose(0, 3) + pose(1, 3) + pose(2, 3);
    }

    inline int get_frame_hash(const cv::Mat& frame) {
        const auto n_channel = frame.channels();

        if (n_channel == 1) {
            return frame.at<uchar>(image_height_ * 0.5, image_width_ * 0.25)
                   + frame.at<uchar>(image_height_ * 0.5, image_width_ * 0.5)
                   + frame.at<uchar>(image_height_ * 0.5, image_width_ * 0.75);
        }
        else {
            return rgb_to_24bit(frame.at<cv::Vec3b>(image_height_ * 0.5, image_width_ * 0.25))
                   + rgb_to_24bit(frame.at<cv::Vec3b>(image_height_ * 0.5, image_width_ * 0.5))
                   + rgb_to_24bit(frame.at<cv::Vec3b>(image_height_ * 0.5, image_width_ * 0.75));
        }
    }

    inline int rgb_to_24bit(const cv::Vec3b& rgb) {
        return rgb[0] * 0x1000 + rgb[0] * 0x10 + rgb[0];
    }

    std::string serialize_as_protobuf(const std::vector<openvslam::data::keyframe*>& keyfrms,
                                      const std::vector<openvslam::data::landmark*>& all_landmarks,
                                      const std::set<openvslam::data::landmark*>& local_landmarks,
                                      const openvslam::Mat44_t& current_camera_pose);

    std::string base64_encode(unsigned char const* bytes_to_encode, unsigned int in_len);
};

} // namespace socket_publisher

#endif // SOCKET_PUBLISHER_DATA_SERIALIZER_H
