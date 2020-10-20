#include "opencv2/core/cuda/common.hpp"
#include "opencv2/core/cuda/utility.hpp"
#include <cuda_runtime.h>
#include "openvslam/cuda/mat_norm.hpp"
#include <helper_cuda.h>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
// #include <Utils.hpp>
#include "openvslam/cuda/Allocator.hpp"

using namespace cv;
using namespace cv::cuda;
using namespace cv::cuda::device;

namespace openvslam { namespace cuda {

__device__ uint8_t d_val;

MatNormGPU::MatNormGPU() {
    // GpuMat should be already stored in memory
    //cudaMallocManaged(&subtract_val, sizeof(uint8_t));
}

MatNormGPU::~MatNormGPU() {
}

__global__
void kernel_get_mat_pixel (uint8_t * src, int w, int step) //uint8_t * pSub)
{
    // assuming that we resized it to CV_32F so the channel number is 1
    //*d_subtract_val = src[ (w*step) + (w)];
    //*pSub = src[ (w * step) + (w)];
    d_val = src[ (w * step) + w];
}

void MatNormGPU::setSubtractValue(const cv::cuda::GpuMat _img, int w)
{
    uint8_t subtract_val;
    //cudaMalloc(&d_subtract_val, sizeof(int));
    //uint8_t * imgPtr;
    //cudaMalloc((void **)&imgPtr, _img.rows*_img.step);
    //cudaMemcpy(imgPtr, _img.ptr<uint8_t>(), _img.rows*_img.step, cudaMemcpyDeviceToDevice);
    //std::cout << "start subtract value" << std::endl;
    //cudaMallocManaged(&subtract_val, sizeof(uint8_t));

    //SET_CLOCK(t0);
    kernel_get_mat_pixel<<<1, 1>>>(_img.data, w, _img.step);
    //SET_CLOCK(t1);
    //cout << TIME_DIFF(t1, t0) << endl;
    //std::cout << "finish kernel command" << std::endl;

    cudaMemcpyFromSymbol(&subtract_val, d_val, sizeof(uint8_t), 0, cudaMemcpyDeviceToHost);
    subMat = cv::cuda::GpuMat(_img.rows, _img.cols, _img.type(), subtract_val, cuda::gpu_mat_allocator);
    //cudaMemcpy(subtract_val, d_subtract_val, sizeof(int), cudaMemcpyDeviceToHost);
    //cudaFree(d_subtract_val);
    //std::cout << "gpu subtract value " << subtract_val << " here" << std::endl;
}

__global__
void kernel_subtract_pixel_from_mat (uint8_t * src, int MaxRows, int MaxCols, int step, int sub)
{
    int row = blockIdx.x * blockDim.x + threadIdx.x; //Row number
    int rowStride = blockDim.x * gridDim.x;
    int col = blockIdx.y * blockDim.y + threadIdx.y; //Column number
    int colStride = blockDim.y * gridDim.y;

    //unsigned int ch = blockIdx.z * blockDim.z + threadIdx.z; //Channel 0
    for (int i = row; i < MaxRows; i += rowStride) {
	for (int j = col; j < MaxCols; j += colStride) {

    	    if (row<MaxRows && col<MaxCols) {
        	int idx = i * step + j; // maxChannels is 1 and ch is 0
        	src[idx] = src[idx] - sub;
    	    }
    	}
    }
}

void MatNormGPU::subtract_pixel_from_mat (cv::cuda::GpuMat _img)
{
    //std::cout << "start subtract pixel " << int(subtract_val) << std::endl;

    //const dim3 block(16, 16);
    //const dim3 grid(((_img.cols + block.x - 1) / block.x), ((_img.rows + block.y - 1)/ block.y));

    //kernel_subtract_pixel_from_mat<<<grid, block>>> (_img.data,_img.rows, _img.cols, _img.step, subtract_val);
    //cv::cuda::subtract(_img, Scalar::all(subtract_val), _img);
    cv::cuda::subtract(_img, subMat, _img);
    //std::cout << "finish subtract pixel" << std::endl;
}

} }
