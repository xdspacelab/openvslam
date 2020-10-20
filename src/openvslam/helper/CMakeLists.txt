# CUDA
# find_package(CUDA QUIET REQUIRED)
# set(CUDA_PROPAGATE_HOST_FLAGS OFF)
# SET(CUDA_HOST_COMPILER /usr/bin/g++)
# set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11" )
# LIST(APPEND CUDA_NVCC_FLAGS "--compiler-options -fno-strict-aliasing -use_fast_math")

# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/cv-helpers.hpp
               ${CMAKE_CURRENT_SOURCE_DIR}/FindType.hpp
               ${CMAKE_CURRENT_SOURCE_DIR}/parallel_for_thread.hpp
               ${CMAKE_CURRENT_SOURCE_DIR}/SGFilter.hpp
               ${CMAKE_CURRENT_SOURCE_DIR}/Utils.hpp
               )

# # Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/helper)
