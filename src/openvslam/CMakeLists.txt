# ----- Find dependencies -----

# spdlog
set(spdlog_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/spdlog/include)

# json
set(json_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/json/include)

# g2o
find_package(g2o REQUIRED
             COMPONENTS
             g2o::core
             g2o::stuff
             g2o::types_sba
             g2o::types_sim3
             g2o::solver_dense
             g2o::solver_eigen
             g2o::solver_csparse
             g2o::csparse_extension
             OPTIONAL_COMPONENTS
             g2o::csparse)

# Check first if CSparse is built from g2o
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

# ----- Create OpenVSLAM library -----

add_library(${PROJECT_NAME}
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

# Set output directory of the library
set_target_properties(${PROJECT_NAME} PROPERTIES
                      OUTPUT_NAME openvslam
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# ----- Compile configuration -----

# OpenMP
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
set(USE_OPENMP OFF CACHE BOOL "Use OpenMP")
if(USE_OPENMP)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_OPENMP)
    message(STATUS "OpenMP: ENABLED")
else()
    message(STATUS "OpenMP: DISABLED")
endif()

set(USE_SSE_ORB OFF CACHE BOOL "Enable SSE3 instruction for ORB extraction")
if(USE_SSE_ORB)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-msse3>)
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_SSE_ORB)
    message(STATUS "SSE3 for ORB extraction: ENABLED")
else()
    message(STATUS "SSE3 for ORB extraction: DISABLED")
endif()

set(USE_SSE_FP_MATH OFF CACHE BOOL "Enable SSE instruction for floating-point operation")
if(USE_SSE_FP_MATH)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-mfpmath=sse>)
    message(STATUS "SSE for floating-point operation: ENABLED")
else()
    message(STATUS "SSE for floating-point operation: DISABLED")
endif()

# Check BoW Framework
set(BOW_FRAMEWORK "DBoW2" CACHE STRING "DBoW2 or FBoW")
set_property(CACHE BOW_FRAMEWORK PROPERTY STRINGS "DBoW2" "FBoW")
if(BOW_FRAMEWORK MATCHES "DBoW2")
    find_package(DBoW2 REQUIRED)
    set(BoW_INCLUDE_DIR ${DBoW2_INCLUDE_DIRS})
    set(BoW_LIBRARY ${DBoW2_LIBS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC USE_DBOW2)
elseif(BOW_FRAMEWORK MATCHES "FBoW")
    find_package(fbow REQUIRED)
    set(BoW_INCLUDE_DIR ${fbow_INCLUDE_DIRS})
    set(BoW_LIBRARY ${fBoW_LIBS})
else()
    message(FATAL_ERROR "Invalid BoW framework: ${BOW_FRAMEWORK}")
endif()
message(STATUS "BoW framework: ${BOW_FRAMEWORK} (found in ${BoW_INCLUDE_DIR})")

# ----- Configure OpenVSLAM library -----

# Include directories
target_include_directories(${PROJECT_NAME}
                           PUBLIC
                           $<BUILD_INTERFACE:${json_INCLUDE_DIR}>
                           $<BUILD_INTERFACE:${spdlog_INCLUDE_DIR}>
                           $<BUILD_INTERFACE:${BoW_INCLUDE_DIR}>
                           $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/>
                           $<INSTALL_INTERFACE:include/openvslam/3rd/json/include>
                           $<INSTALL_INTERFACE:include/openvslam/3rd/spdlog/include>
                           $<INSTALL_INTERFACE:${BoW_INCLUDE_DIR}>
                           $<INSTALL_INTERFACE:include/>)

# Link to required libraries
target_link_libraries(${PROJECT_NAME}
                      PUBLIC
                      Threads::Threads
                      OpenMP::OpenMP_CXX
                      Eigen3::Eigen
                      yaml-cpp
                      opencv_core
                      opencv_features2d
                      opencv_calib3d
                      g2o::core
                      g2o::stuff
                      g2o::types_sba
                      g2o::types_sim3
                      g2o::solver_dense
                      g2o::solver_eigen
                      g2o::solver_csparse
                      g2o::csparse_extension
                      ${CXSPARSE_LIBRARIES}
                      ${SUITESPARSE_LIBRARIES}
                      ${BoW_LIBRARY}
                      PRIVATE
                      ${LAPACK_LIBRARIES})

# ----- Install configuration -----

set(OPENVSLAM_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/openvslam)

# Install OpenVSLAM library
install(TARGETS ${PROJECT_NAME}
        EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
        RUNTIME DESTINATION ${RUNTIME_DESTINATION}
        LIBRARY DESTINATION ${LIBRARY_DESTINATION}
        ARCHIVE DESTINATION ${ARCHIVE_DESTINATION})

# Install OpenVSLAM headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR})

# Install 3rd party headers
install(FILES ${json_INCLUDE_DIR}/nlohmann/json.hpp ${json_INCLUDE_DIR}/nlohmann/json_fwd.hpp
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/3rd/json/include/nlohmann)
install(DIRECTORY ${spdlog_INCLUDE_DIR}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/3rd/spdlog)

# ----- Append subdirectory -----

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
