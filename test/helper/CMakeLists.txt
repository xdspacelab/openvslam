# create test helper library
add_library(test_helper SHARED
        bearing_vector.cc
        keypoint.cc
        landmark.cc)
# add include directory as PUBLIC (because the headers are included in test codes)
target_include_directories(test_helper SYSTEM
        PUBLIC ${PROJECT_SOURCE_DIR}/test
        ${PROJECT_SOURCE_DIR}/src)
