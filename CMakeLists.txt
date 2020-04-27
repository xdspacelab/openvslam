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

# ----- Set build type -----

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release")
endif()
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

option(BUILD_SHARED_LIBS "Build OpenVSLAM as a shared library" ON)

# ----- Set options for debugging -----

set(USE_CCACHE ON CACHE BOOL "Use ccache to accelerate build")
find_program(CCACHE_EXE ccache)
if(USE_CCACHE AND CCACHE_EXE AND NOT MSVC)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_EXE}")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "${CCACHE_EXE}")
    message(STATUS "ccache: Enabled")
else()
    set(USE_CCACHE OFF)
    message(STATUS "ccache: Disabled")
endif()

set(USE_SANITIZER OFF CACHE BOOL "Enable Address/Memory sanitizer (set env as ASAN_OPTIONS=detect_leaks=1)")
if(USE_SANITIZER AND NOT MSVC)
    add_compile_options(-fno-omit-frame-pointer -fsanitize=address)
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_STATIC_LINKER_FLAGS "${CMAKE_STATIC_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
    message(STATUS "Address/Memory sanitizer: ENABLED")
else()
    message(STATUS "Address/Memory sanitizer: DISABLED")
endif()

# ----- Set compiler options -----

# Note: add_compile_options() accepts only "simple" generator-expressions :-(
if(MSVC)
    # C4251: DLL export, C4244: floating->integer, C4305: double->float, C4267: size_t->any, C4127: constant condition
    add_compile_options(/W4 /MT /source-charset:utf-8 /execution-charset:utf-8 /wd4251 /wd4244 /wd4305 /wd4267 /wd4127)
    add_compile_options("$<$<CONFIG:Debug>:/Zo>;$<$<CONFIG:Release>:/O2>")
else()
    add_compile_options(-Wall -Wextra)
    add_compile_options("$<$<CONFIG:Debug>:-Og>;$<$<CONFIG:Release>:-O3;-ffast-math>")
endif()

set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Enable architecture-aware optimization")
if(BUILD_WITH_MARCH_NATIVE AND NOT MSVC)
    add_compile_options(-mtune=native -march=native)
    message(STATUS "Architecture-aware optimization: ENABLED")
else()
    message(STATUS "Architecture-aware optimization: DISABLED")
endif()

# ----- Find dependencies -----

# Threads
find_package(Threads REQUIRED)

# OpenMP
find_package(OpenMP REQUIRED)
if(NOT TARGET OpenMP::OpenMP_CXX)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

# Eigen
find_package(Eigen3 3.3 REQUIRED)

# yaml-cpp
find_package(yaml-cpp REQUIRED)

# OpenCV
find_package(OpenCV 3.3.1 QUIET
             COMPONENTS
             core imgcodecs videoio features2d calib3d highgui)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 REQUIRED
                 COMPONENTS
                 core imgcodecs videoio features2d calib3d highgui)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

# ----- Install configuration -----

include(CMakePackageConfigHelpers)

# Generate cmake configuration scripts
set(OPENVSLAM_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(OPENVSLAM_VERSION_CONFIG ${OPENVSLAM_GENERATED_DIR}/${PROJECT_NAME}ConfigVersion.cmake)
set(OPENVSLAM_PROJECT_CONFIG ${OPENVSLAM_GENERATED_DIR}/${PROJECT_NAME}Config.cmake)
set(OPENVSLAM_TARGETS_EXPORT_NAME ${PROJECT_NAME}Targets)
set(OPENVSLAM_CONFIG_INSTALL_DIR lib/cmake/${PROJECT_NAME})
set(OPENVSLAM_NAMESPACE "${PROJECT_NAME}::")
set(OPENVSLAM_VERSION 1.0.0)

# Create a version config file
write_basic_package_version_file(${OPENVSLAM_VERSION_CONFIG}
                                 VERSION ${OPENVSLAM_VERSION}
                                 COMPATIBILITY SameMajorVersion)
# Create a project config file
configure_file(${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in ${OPENVSLAM_PROJECT_CONFIG} @ONLY)

# Install to the specified directory
install(FILES ${OPENVSLAM_VERSION_CONFIG} ${OPENVSLAM_PROJECT_CONFIG}
        DESTINATION ${OPENVSLAM_CONFIG_INSTALL_DIR})
install(EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
        NAMESPACE ${OPENVSLAM_NAMESPACE}
        DESTINATION ${OPENVSLAM_CONFIG_INSTALL_DIR})

# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)

# ----- Build selection -----

set(USE_PANGOLIN_VIEWER ON CACHE BOOL "Enable Pangolin Viewer")
set(USE_SOCKET_PUBLISHER OFF CACHE BOOL "Enable Socket Publisher")

add_subdirectory(src)

set(BUILD_EXAMPLES ON CACHE BOOL "Build examples")
if(BUILD_EXAMPLES)
    add_subdirectory(example)
endif()

set(BUILD_TESTS OFF CACHE BOOL "Build tests")
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif()
