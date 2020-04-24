#ifndef OPENVSLAM_OPTIMIZE_G2O_SIM3_MUTUAL_REPROJ_EDGE_WRAPPER_H
#define OPENVSLAM_OPTIMIZE_G2O_SIM3_MUTUAL_REPROJ_EDGE_WRAPPER_H

#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/camera/equirectangular.h"
#include "openvslam/camera/radial_division.h"
#include "openvslam/data/landmark.h"
#include "openvslam/optimize/internal/sim3/forward_reproj_edge.h"
#include "openvslam/optimize/internal/sim3/backward_reproj_edge.h"

#include <g2o/core/robust_kernel_impl.h>

namespace openvslam {

namespace data {
class landmark;
} // namespace data

namespace optimize {
namespace internal {
namespace sim3 {

template<typename T>
class mutual_reproj_edge_wapper {
public:
    mutual_reproj_edge_wapper() = delete;

    mutual_reproj_edge_wapper(T* shot1, unsigned int idx1, data::landmark* lm1,
                              T* shot2, unsigned int idx2, data::landmark* lm2,
                              internal::sim3::transform_vertex* Sim3_12_vtx, const float sqrt_chi_sq);

    bool is_inlier() const;

    bool is_outlier() const;

    void set_as_inlier() const;

    void set_as_outlier() const;

    //! keyfrm_2で観測している3次元点をkeyfrm_1に再投影するconstraint edge
    //! (カメラモデルはkeyfrm_1のものに従う)
    base_forward_reproj_edge* edge_12_;
    //! keyfrm_1で観測している3次元点をkeyfrm_2に再投影するconstraint edge
    //! (カメラモデルはkeyfrm_2のものに従う)
    base_backward_reproj_edge* edge_21_;

    T *shot1_, *shot2_;
    unsigned int idx1_, idx2_;
    data::landmark *lm1_, *lm2_;
};

template<typename T>
inline mutual_reproj_edge_wapper<T>::mutual_reproj_edge_wapper(T* shot1, unsigned int idx1, data::landmark* lm1,
                                                               T* shot2, unsigned int idx2, data::landmark* lm2,
                                                               internal::sim3::transform_vertex* Sim3_12_vtx, const float sqrt_chi_sq)
    : shot1_(shot1), shot2_(shot2), idx1_(idx1), idx2_(idx2), lm1_(lm1), lm2_(lm2) {
    // 1. forward edgeを作成
    {
        camera::base* camera1 = shot1->camera_;

        switch (camera1->model_type_) {
            case camera::model_type_t::Perspective: {
                auto c = static_cast<camera::perspective*>(camera1);

                // 3次元点はkeyfrm_2で観測しているもの，カメラモデルと特徴点はkeyfrm_1のもの
                auto edge_12 = new internal::sim3::perspective_forward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_1 = shot1->undist_keypts_.at(idx1);
                const Vec2_t obs_1{undist_keypt_1.pt.x, undist_keypt_1.pt.y};
                const float inv_sigma_sq_1 = shot1->inv_level_sigma_sq_.at(undist_keypt_1.octave);
                edge_12->setMeasurement(obs_1);
                edge_12->setInformation(Mat22_t::Identity() * inv_sigma_sq_1);
                // 3次元点をセット
                edge_12->pos_w_ = lm2->get_pos_in_world();
                // カメラモデルをセット
                edge_12->fx_ = c->fx_;
                edge_12->fy_ = c->fy_;
                edge_12->cx_ = c->cx_;
                edge_12->cy_ = c->cy_;

                edge_12->setVertex(0, Sim3_12_vtx);

                edge_12_ = edge_12;
                break;
            }
            case camera::model_type_t::Fisheye: {
                auto c = static_cast<camera::fisheye*>(camera1);

                // 3次元点はkeyfrm_2で観測しているもの，カメラモデルと特徴点はkeyfrm_1のもの
                auto edge_12 = new internal::sim3::perspective_forward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_1 = shot1->undist_keypts_.at(idx1);
                const Vec2_t obs_1{undist_keypt_1.pt.x, undist_keypt_1.pt.y};
                const float inv_sigma_sq_1 = shot1->inv_level_sigma_sq_.at(undist_keypt_1.octave);
                edge_12->setMeasurement(obs_1);
                edge_12->setInformation(Mat22_t::Identity() * inv_sigma_sq_1);
                // 3次元点をセット
                edge_12->pos_w_ = lm2->get_pos_in_world();
                // カメラモデルをセット
                edge_12->fx_ = c->fx_;
                edge_12->fy_ = c->fy_;
                edge_12->cx_ = c->cx_;
                edge_12->cy_ = c->cy_;

                edge_12->setVertex(0, Sim3_12_vtx);

                edge_12_ = edge_12;
                break;
            }
            case camera::model_type_t::Equirectangular: {
                auto c = static_cast<camera::equirectangular*>(camera1);

                // 3次元点はkeyfrm_2で観測しているもの，カメラモデルと特徴点はkeyfrm_1のもの
                auto edge_12 = new internal::sim3::equirectangular_forward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_1 = shot1->undist_keypts_.at(idx1);
                const Vec2_t obs_1{undist_keypt_1.pt.x, undist_keypt_1.pt.y};
                const float inv_sigma_sq_1 = shot1->inv_level_sigma_sq_.at(undist_keypt_1.octave);
                edge_12->setMeasurement(obs_1);
                edge_12->setInformation(Mat22_t::Identity() * inv_sigma_sq_1);
                // 3次元点をセット
                edge_12->pos_w_ = lm2->get_pos_in_world();
                // カメラモデルをセット
                edge_12->cols_ = c->cols_;
                edge_12->rows_ = c->rows_;

                edge_12->setVertex(0, Sim3_12_vtx);

                edge_12_ = edge_12;
                break;
            }
            case camera::model_type_t::RadialDivision: {
                auto c = static_cast<camera::radial_division*>(camera1);

                // 3次元点はkeyfrm_2で観測しているもの，カメラモデルと特徴点はkeyfrm_1のもの
                auto edge_12 = new internal::sim3::perspective_forward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_1 = shot1->undist_keypts_.at(idx1);
                const Vec2_t obs_1{undist_keypt_1.pt.x, undist_keypt_1.pt.y};
                const float inv_sigma_sq_1 = shot1->inv_level_sigma_sq_.at(undist_keypt_1.octave);
                edge_12->setMeasurement(obs_1);
                edge_12->setInformation(Mat22_t::Identity() * inv_sigma_sq_1);
                // 3次元点をセット
                edge_12->pos_w_ = lm2->get_pos_in_world();
                // カメラモデルをセット
                edge_12->fx_ = c->fx_;
                edge_12->fy_ = c->fy_;
                edge_12->cx_ = c->cx_;
                edge_12->cy_ = c->cy_;

                edge_12->setVertex(0, Sim3_12_vtx);

                edge_12_ = edge_12;
                break;
            }
        }

        // loss functionを設定
        auto huber_kernel_12 = new g2o::RobustKernelHuber();
        huber_kernel_12->setDelta(sqrt_chi_sq);
        edge_12_->setRobustKernel(huber_kernel_12);
    }

    // 2. backward edgeを作成
    {
        camera::base* camera2 = shot2->camera_;

        switch (camera2->model_type_) {
            case camera::model_type_t::Perspective: {
                auto c = static_cast<camera::perspective*>(camera2);

                // 3次元点はkeyfrm_1で観測しているもの，カメラモデルと特徴点はkeyfrm_2のもの
                auto edge_21 = new internal::sim3::perspective_backward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_2 = shot2->undist_keypts_.at(idx2);
                const Vec2_t obs_2{undist_keypt_2.pt.x, undist_keypt_2.pt.y};
                const float inv_sigma_sq_2 = shot2->inv_level_sigma_sq_.at(undist_keypt_2.octave);
                edge_21->setMeasurement(obs_2);
                edge_21->setInformation(Mat22_t::Identity() * inv_sigma_sq_2);
                // 3次元点をセット
                edge_21->pos_w_ = lm1->get_pos_in_world();
                // カメラモデルをセット
                edge_21->fx_ = c->fx_;
                edge_21->fy_ = c->fy_;
                edge_21->cx_ = c->cx_;
                edge_21->cy_ = c->cy_;

                edge_21->setVertex(0, Sim3_12_vtx);

                edge_21_ = edge_21;
                break;
            }
            case camera::model_type_t::Fisheye: {
                auto c = static_cast<camera::fisheye*>(camera2);

                // 3次元点はkeyfrm_1で観測しているもの，カメラモデルと特徴点はkeyfrm_2のもの
                auto edge_21 = new internal::sim3::perspective_backward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_2 = shot2->undist_keypts_.at(idx2);
                const Vec2_t obs_2{undist_keypt_2.pt.x, undist_keypt_2.pt.y};
                const float inv_sigma_sq_2 = shot2->inv_level_sigma_sq_.at(undist_keypt_2.octave);
                edge_21->setMeasurement(obs_2);
                edge_21->setInformation(Mat22_t::Identity() * inv_sigma_sq_2);
                // 3次元点をセット
                edge_21->pos_w_ = lm1->get_pos_in_world();
                // カメラモデルをセット
                edge_21->fx_ = c->fx_;
                edge_21->fy_ = c->fy_;
                edge_21->cx_ = c->cx_;
                edge_21->cy_ = c->cy_;

                edge_21->setVertex(0, Sim3_12_vtx);

                edge_21_ = edge_21;
                break;
            }
            case camera::model_type_t::Equirectangular: {
                auto c = static_cast<camera::equirectangular*>(camera2);

                // 3次元点はkeyfrm_1で観測しているもの，カメラモデルと特徴点はkeyfrm_2のもの
                auto edge_21 = new internal::sim3::equirectangular_backward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_2 = shot2->undist_keypts_.at(idx2);
                const Vec2_t obs_2{undist_keypt_2.pt.x, undist_keypt_2.pt.y};
                const float inv_sigma_sq_2 = shot2->inv_level_sigma_sq_.at(undist_keypt_2.octave);
                edge_21->setMeasurement(obs_2);
                edge_21->setInformation(Mat22_t::Identity() * inv_sigma_sq_2);
                // 3次元点をセット
                edge_21->pos_w_ = lm1->get_pos_in_world();
                // カメラモデルをセット
                edge_21->cols_ = c->cols_;
                edge_21->rows_ = c->rows_;

                edge_21->setVertex(0, Sim3_12_vtx);

                edge_21_ = edge_21;
                break;
            }
            case camera::model_type_t::RadialDivision: {
                auto c = static_cast<camera::radial_division*>(camera2);

                // 3次元点はkeyfrm_1で観測しているもの，カメラモデルと特徴点はkeyfrm_2のもの
                auto edge_21 = new internal::sim3::perspective_backward_reproj_edge();
                // 特徴点情報と再投影誤差分散をセット
                const auto& undist_keypt_2 = shot2->undist_keypts_.at(idx2);
                const Vec2_t obs_2{undist_keypt_2.pt.x, undist_keypt_2.pt.y};
                const float inv_sigma_sq_2 = shot2->inv_level_sigma_sq_.at(undist_keypt_2.octave);
                edge_21->setMeasurement(obs_2);
                edge_21->setInformation(Mat22_t::Identity() * inv_sigma_sq_2);
                // 3次元点をセット
                edge_21->pos_w_ = lm1->get_pos_in_world();
                // カメラモデルをセット
                edge_21->fx_ = c->fx_;
                edge_21->fy_ = c->fy_;
                edge_21->cx_ = c->cx_;
                edge_21->cy_ = c->cy_;

                edge_21->setVertex(0, Sim3_12_vtx);

                edge_21_ = edge_21;
                break;
            }
        }

        // loss functionを設定
        auto huber_kernel_21 = new g2o::RobustKernelHuber();
        huber_kernel_21->setDelta(sqrt_chi_sq);
        edge_21_->setRobustKernel(huber_kernel_21);
    }
}

template<typename T>
inline bool mutual_reproj_edge_wapper<T>::is_inlier() const {
    return edge_12_->level() == 0 && edge_21_->level() == 0;
}

template<typename T>
inline bool mutual_reproj_edge_wapper<T>::is_outlier() const {
    return !is_inlier();
}

template<typename T>
inline void mutual_reproj_edge_wapper<T>::set_as_inlier() const {
    edge_12_->setLevel(0);
    edge_21_->setLevel(0);
}

template<typename T>
inline void mutual_reproj_edge_wapper<T>::set_as_outlier() const {
    edge_12_->setLevel(1);
    edge_21_->setLevel(1);
}

} // namespace sim3
} // namespace internal
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_G2O_SIM3_MUTUAL_REPROJ_EDGE_WRAPPER_H
