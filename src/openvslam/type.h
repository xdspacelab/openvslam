#ifndef OPENVSLAM_TYPE_H
#define OPENVSLAM_TYPE_H

#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/types.hpp>

#ifndef M_PI
// M_PI is not part of the C++ standard. Rather it is part of the POSIX standard. As such,
// it is not directly available on Visual C++ (although _USE_MATH_DEFINES does exist).
#define M_PI 3.14159265358979323846
#endif

namespace openvslam {

// floating point type

typedef float real_t;

// Eigen matrix types

template<size_t R, size_t C>
using MatRC_t = Eigen::Matrix<double, R, C>;

using Mat22_t = Eigen::Matrix2d;

using Mat33_t = Eigen::Matrix3d;

using Mat44_t = Eigen::Matrix4d;

using Mat55_t = MatRC_t<5, 5>;

using Mat66_t = MatRC_t<6, 6>;

using Mat77_t = MatRC_t<7, 7>;

using Mat34_t = MatRC_t<3, 4>;

using MatX_t = Eigen::MatrixXd;

// Eigen vector types

template<size_t R>
using VecR_t = Eigen::Matrix<double, R, 1>;

using Vec2_t = Eigen::Vector2d;

using Vec3_t = Eigen::Vector3d;

using Vec4_t = Eigen::Vector4d;

using Vec5_t = VecR_t<5>;

using Vec6_t = VecR_t<6>;

using Vec7_t = VecR_t<7>;

using VecX_t = Eigen::VectorXd;

// Eigen Quaternion type

using Quat_t = Eigen::Quaterniond;

// STL with Eigen custom allocator

template<typename T>
using eigen_alloc_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template<typename T, typename U>
using eigen_alloc_map = std::map<T, U, std::less<T>, Eigen::aligned_allocator<std::pair<const T, U>>>;

template<typename T>
using eigen_alloc_set = std::set<T, std::less<T>, Eigen::aligned_allocator<const T>>;

template<typename T, typename U>
using eigen_alloc_unord_map = std::unordered_map<T, U, std::hash<T>, std::equal_to<T>, Eigen::aligned_allocator<std::pair<const T, U>>>;

template<typename T>
using eigen_alloc_unord_set = std::unordered_set<T, std::hash<T>, std::equal_to<T>, Eigen::aligned_allocator<const T>>;

// vector operators

template<typename T>
inline Vec2_t operator+(const Vec2_t& v1, const cv::Point_<T>& v2) {
    return {v1(0) + v2.x, v1(1) + v2.y};
}

template<typename T>
inline Vec2_t operator+(const cv::Point_<T>& v1, const Vec2_t& v2) {
    return v2 + v1;
}

template<typename T>
inline Vec2_t operator-(const Vec2_t& v1, const cv::Point_<T>& v2) {
    return v1 + (-v2);
}

template<typename T>
inline Vec2_t operator-(const cv::Point_<T>& v1, const Vec2_t& v2) {
    return v1 + (-v2);
}

} // namespace openvslam

#endif // OPENVSLAM_TYPE_H
