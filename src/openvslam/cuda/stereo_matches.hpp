#pragma once
#ifndef __STEREOMATCH_HPP__
#define __STEREOMATCH_HPP__

#include <vector>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <cuda_runtime.h>
#include "openvslam/cuda/Cuda.hpp"

namespace openvslam { namespace cuda {
  using namespace std;
  using namespace cv;
  using namespace cv::cuda;

  class StereoMatching {
    unsigned int maxKeypoints;
    KeyPoint * mvKeys;
    KeyPoint * mvKeysRight; 
    GpuMat mDescriptors;
    GpuMat desc;

    GpuMat mDescriptorsRight;
    GpuMat descRight;

    cudaStream_t stream;
    Stream cvStream;
    float scale; 
  public:
    StereoMatching(int maxKeypoints = 10000);
    ~StereoMatching();

    
  };
} }
#endif
