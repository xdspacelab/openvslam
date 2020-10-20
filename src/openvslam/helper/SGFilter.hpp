// SGFilter.h 
// 
// Implenetation of the Savitzky-Golay Filter in cpp (Arduino/Teensy) 
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <unistd.h>
#include <iostream>

namespace openvslam
{

    class SGFilter
    {
	public:
        struct DATA 
		{
	    float yRaw = 0.0F;
            float y = 0.0F;
            float yDot = 0.0F;
		};

        // Methods
        SGFilter(uint32_t poly_order, uint32_t filter_size);

        void update(float new_time_float, float new_val);
        void debug_print_A(void);
        float powFast(float x, uint32_t n);    

        // Attributes
        uint32_t poly_order_;
        uint32_t filter_size_;

        uint32_t filter_iter_;
        uint32_t current_line_;

        float zero_time_;
        float filter_value_;

        Eigen::VectorXf f_; // Vector of function values 
        Eigen::VectorXf t_; // Vector of time slots
        Eigen::MatrixXf A_; // Matrix of Least-Squares coefficients 
        Eigen::VectorXf c_; // Vector of Polynomial coefficients
        DATA data_;
    };
}