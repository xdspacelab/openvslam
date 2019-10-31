####################
# 3rd Part headers #
####################

# spdlog
set(spdlog_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/spdlog/include)

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

# json
set(json_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/json/include)

#####################
# OpenVSlam library #
#####################

# Create the library
add_library(${PROJECT_NAME} SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/config.h
        ${CMAKE_CURRENT_SOURCE_DIR}/type.h
        ${CMAKE_CURRENT_SOURCE_DIR}/system.h
        ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.h
        ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.h
        ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.h
        ${CMAKE_CURRENT_SOURCE_DIR}/config.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/system.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.cc)

# Append all sources
add_subdirectory(camera)
add_subdirectory(data)
add_subdirectory(feature)
add_subdirectory(initialize)
add_subdirectory(io)
add_subdirectory(match)
add_subdirectory(module)
add_subdirectory(optimize)
add_subdirectory(publish)
add_subdirectory(solve)
add_subdirectory(util)

set_target_properties(${PROJECT_NAME} PROPERTIES
        OUTPUT_NAME openvslam
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# Set Compilation definitions if needed
if(USE_OPENMP)
    find_package(OpenMP)
    if(OpenMP_FOUND)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)
        message(STATUS "OpenMP: ENABLED")
    else()
        message(STATUS "OpenMP: DISABLED")
    endif()
else()
    message(STATUS "OpenMP: DISABLED")
endif()

if(USE_SSE_ORB)
    message(STATUS "SSE3 for ORB extraction (-msse3): ENABLED")
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_SSE_ORB)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -msse3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3")
else()
    message(STATUS "SSE3 for ORB extraction (-msse3): DISABLED")
endif()

if(USE_SSE_FP_MATH)
    message(STATUS "SSE for floating-point operation (-mfpmath=sse): ENABLED")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mfpmath=sse")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=sse")
else()
    message(STATUS "SSE for floating-point operation (-mfpmath=sse): DISABLED")
endif()

# Check BoW Framework
if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
    set(BOW_INCLUDE_DIRS ${DBoW2_INCLUDE_DIRS})
    set(BOW_LIBS ${DBoW2_LIBS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DBOW2 )
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow REQUIRED)
    set(BOW_INCLUDE_DIRS ${fbow_INCLUDE_DIRS})
    set(BOW_LIBS ${fbow_LIBS})
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()

message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${BOW_INCLUDE_DIRS})")

# Include directories
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${json_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include/openvslam/3rd/json/include>
  $<INSTALL_INTERFACE:include/openvslam/3rd/spdlog/include>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>
  $<INSTALL_INTERFACE:include/>
  $<BUILD_INTERFACE:${popl_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${spdlog_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${BOW_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:${BOW_INCLUDE_DIRS}>
)

# Link to required libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
        Threads::Threads
        Eigen3::Eigen
        opencv_core
        opencv_features2d
        opencv_calib3d
        yaml-cpp
        g2o::core
        g2o::stuff
        g2o::solver_dense
        g2o::solver_eigen
        g2o::solver_csparse
        g2o::csparse_extension
        g2o::types_sba
        g2o::types_sim3
        ${CXSPARSE_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${BOW_LIBS}
        PRIVATE
        ${LAPACK_LIBRARIES}
        )

###########
# Install #
###########

# Install OpenVSlam library

install(TARGETS ${PROJECT_NAME}
      EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
      RUNTIME DESTINATION ${RUNTIME_DESTINATION}
      LIBRARY DESTINATION ${LIBRARY_DESTINATION}
      ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
    )

# Install OpenVSlam headers
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR})

# Install public 3rd party headers
install(FILES "${json_INCLUDE_DIR}/nlohmann/json.hpp" DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/3rd/json/include/nlohmann/)
install(DIRECTORY ${spdlog_INCLUDE_DIR} DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/3rd/spdlog/)

