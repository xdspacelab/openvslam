cmake_minimum_required(VERSION 3.1)
project(openvslam LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0042)
    cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0074)
    cmake_policy(SET CMP0074 OLD)
endif()

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

##################
# Set build type #
##################

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

###########################
# Set application options #
###########################

set(USE_CCACHE ON CACHE BOOL "Use ccache to accelerate a build")

find_program(CCACHE_EXE ccache)
if(USE_CCACHE AND CCACHE_EXE)
    message(STATUS "ccache: Enabled")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_EXE}")
else()
    message(STATUS "ccache: Disabled")
    set(USE_CCACHE OFF)
endif()

set(USE_SANITIZER OFF CACHE BOOL "Enable Address/Memory sanitizer (set env as ASAN_OPTIONS=detect_leaks=1)")

if(USE_SANITIZER)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    message(STATUS "Address/Memory sanitizer: ENABLED")
else()
    message(STATUS "Address/Memory sanitizer: DISABLED")
endif()

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of google-glog")

if(USE_STACK_TRACE_LOGGER)
    message(STATUS "Stack trace logger: ENABLED")
    # Glog
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

set(USE_GOOGLE_PERFTOOLS OFF CACHE BOOL "Enable profiler of google-perftools")

if(USE_GOOGLE_PERFTOOLS)
    message(STATUS "Google Perftools: ENABLED")
    # Gperftools
    find_package(Gperftools REQUIRED)
    include_directories(${GPERFTOOLS_INCLUDE_DIRS})
else()
    message(STATUS "Google Perftools: DISABLED")
endif()

########################
# Set compiler options #
########################

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Og")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Og")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -ffast-math")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -ffast-math")

set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization (-march=native)")

if(BUILD_WITH_MARCH_NATIVE)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native -march=native")
    message(STATUS "Architecture-aware optimization (-march=native): ENABLED")
else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -mtune=native")
    message(STATUS "Architecture-aware optimization (-march=native): DISABLED")
endif()

set(USE_SSE_ORB OFF CACHE BOOL "Enable SSE3 instruction for ORB extraction (-msse3)")

set(USE_SSE_FP_MATH OFF CACHE BOOL "Enable SSE instruction for floating-point operation (-mfpmath=sse)")


#################
# Find packages #
#################

# thread library
find_package(Threads REQUIRED)

# OpenMP
set(USE_OPENMP ON CACHE BOOL "Use OpenMP")

# OpenCV
find_package(OpenCV 3.3.1 COMPONENTS core features2d calib3d)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 REQUIRED COMPONENTS core features2d calib3d)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# Eigen
find_package(Eigen3 3.2 REQUIRED)

# Find imported target from g2o
find_package(g2o REQUIRED COMPONENTS
    g2o::core
    g2o::stuff
    g2o::solver_dense
    g2o::solver_eigen
    g2o::solver_csparse
    g2o::csparse_extension
    g2o::types_sba
    g2o::types_sim3
    OPTIONAL_COMPONENTS
    g2o::csparse)

# Check first if csparse is build from g2o.
if(TARGET g2o::csparse)
    set(${CXSPARSE_LIBRARIES} g2o::csparse)
else()
    # CXSparse for g2o
    find_package(CXSparse)
    if(CXSPARSE_FOUND)
        include_directories(${CXSPARSE_INCLUDE_DIRS})
    endif()

    # SuiteSparse for g2o
    find_package(SuiteSparse)
    if(SUITESPARSE_FOUND)
        include_directories(${SUITESPARSE_INCLUDE_DIRS})
    endif()

endif()

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# pangolin viewer
set(USE_PANGOLIN_VIEWER ON CACHE BOOL "Enable Pangolin Viewer")
if(USE_PANGOLIN_VIEWER)
    find_package(Pangolin REQUIRED)
endif()

# socket publisher
set(USE_SOCKET_PUBLISHER OFF CACHE BOOL "Enable Socket Publisher")
if(USE_SOCKET_PUBLISHER)
    find_package(sioclient REQUIRED)
    find_package(Protobuf REQUIRED)
    if(NOT PROTOBUF_PROTOC_EXECUTABLE)
        message(FATAL_ERROR "Could not find protoc executable (PROTOBUF_PROTOC_EXECUTABLE)")
    endif()
    message(STATUS "Found protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")
endif()

#################
# BoW framework #
#################

set(BOW_FRAMEWORK "DBoW2" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")

#################
# Config files  #
#################

# Generate cmake configuration scripts

set(OPENVSLAM_GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(OPENVSLAM_VERSION_CONFIG "${OPENVSLAM_GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake")
set(OPENVSLAM_PROJECT_CONFIG "${OPENVSLAM_GENERATED_DIR}/${PROJECT_NAME}Config.cmake")
set(OPENVSLAM_TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
set(OPENVSLAM_CONFIG_INSTALL_DIR "lib/cmake/${PROJECT_NAME}")
set(OPENVSLAM_NAMESPACE "${PROJECT_NAME}::")
set(OPENVSLAM_VERSION 1.0.0)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    "${OPENVSLAM_VERSION_CONFIG}" VERSION ${OPENVSLAM_VERSION} COMPATIBILITY SameMajorVersion
    )

configure_file("${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in" "${OPENVSLAM_PROJECT_CONFIG}" @ONLY)

install(
    FILES "${OPENVSLAM_PROJECT_CONFIG}" "${OPENVSLAM_VERSION_CONFIG}"
    DESTINATION "${OPENVSLAM_CONFIG_INSTALL_DIR}")

install(
    EXPORT "${OPENVSLAM_TARGETS_EXPORT_NAME}"
    NAMESPACE "${OPENVSLAM_NAMESPACE}"
    DESTINATION "${OPENVSLAM_CONFIG_INSTALL_DIR}")

# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(OPENVSLAM_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/openvslam)
set(PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_viewer)
set(SOCKER_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/socket_viewer)

###################
# Build Selection #
###################

add_subdirectory(src)

set(BUILD_EXAMPLES OFF CACHE BOOL "Build Examples")

if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

set(BUILD_TESTS OFF CACHE BOOL "Build tests")
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
