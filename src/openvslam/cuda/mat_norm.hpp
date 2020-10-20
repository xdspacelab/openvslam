#pragma once
#ifndef __MATNORM_HPP__
#define __MATNORM_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <cuda_runtime.h>
#include "openvslam/cuda/Cuda.hpp"

namespace openvslam { namespace cuda {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  class MatNormGPU {
    //GpuMat gMat;

    //uint8_t subtract_val;
    GpuMat subMat; 

  public:
    MatNormGPU();
    ~MatNormGPU();

    void setSubtractValue(cv::cuda::GpuMat _img, int w);
    void subtract_pixel_from_mat (cv::cuda::GpuMat _img);
  };
} }
#endif
