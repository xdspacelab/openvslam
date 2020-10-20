
#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/utility.hpp"
#include "opencv2/core/cuda/reduce.hpp"
#include "opencv2/core/cuda/functional.hpp"
#include <helper_cuda.h>
#include "openvslam/cuda/stereo_matches.hpp"
// #include <Utils.hpp>

using namespace cv;
using namespace cv::cuda;
using namespace cv::cuda::device;

namespace openvslam { namespace cuda {

  StereoMatching::StereoMatching(int w) : maxKeypoints(maxKeypoints), mDescriptors(maxKeypoints, 32, CV_8UC1), mDescriptorsRight(maxKeypoints, 32, CV_8UC1) {
    checkCudaErrors( cudaStreamCreate(&stream) );
    cvStream = StreamAccessor::wrapStream(stream);
    checkCudaErrors( cudaMalloc(&mvKeys, sizeof(KeyPoint) * maxKeypoints) );
    checkCudaErrors( cudaMalloc(&mvKeysRight, sizeof(KeyPoint) * maxKeypoints) );
  }

  StereoMatching::~StereoMatching() {
    cvStream.~Stream();
    checkCudaErrors( cudaFree(mvKeys) );
    checkCudaErrors( cudaFree(mvKeysRight) );
    checkCudaErrors( cudaStreamDestroy(stream) );
  }



} }
