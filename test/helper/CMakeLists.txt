# Create test helper library
add_library(test_helper
            bearing_vector.h
            keypoint.h
            landmark.h
            bearing_vector.cc
            keypoint.cc
            landmark.cc)

# Add include directory as PUBLIC (because the headers are included in test codes)
target_include_directories(test_helper
                           PUBLIC
                           ${PROJECT_SOURCE_DIR}/test
                           ${PROJECT_SOURCE_DIR}/src)

# Link to required libraries
target_link_libraries(test_helper
                      PUBLIC
                      Eigen3::Eigen
                      opencv_core)
