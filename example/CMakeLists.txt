# ----- Find dependencies -----

# popl
set(popl_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/popl/include)

set(USE_STACK_TRACE_LOGGER OFF CACHE BOOL "Enable automatic stack trace logger of google-glog")
if(USE_STACK_TRACE_LOGGER)
    find_package(Glog REQUIRED)
    include_directories(${GLOG_INCLUDE_DIR})
    message(STATUS "Stack trace logger: ENABLED")
else()
    message(STATUS "Stack trace logger: DISABLED")
endif()

set(USE_GOOGLE_PERFTOOLS OFF CACHE BOOL "Enable profiler of google-perftools")
if(USE_GOOGLE_PERFTOOLS)
    find_package(Gperftools REQUIRED)
    include_directories(${GPERFTOOLS_INCLUDE_DIRS})
    message(STATUS "Google Perftools: ENABLED")
else()
    message(STATUS "Google Perftools: DISABLED")
endif()

# SFML include directory
set(sfml_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/3rd/SFML/include)

# ----- Show dialog -----

if(USE_PANGOLIN_VIEWER)
    message(STATUS "Viewer for examples: PangolinViewer")
elseif(USE_SOCKET_PUBLISHER)
    message(STATUS "Viewer for examples: SocketPublisher")
else()
    message(STATUS "Viewer for examples: None")
endif()

# ----- Build example executables -----

set(EXECUTABLE_TARGETS "")

add_executable(run_camera_slam run_camera_slam.cc)
list(APPEND EXECUTABLE_TARGETS run_camera_slam)

add_executable(run_camera_localization run_camera_localization.cc)
list(APPEND EXECUTABLE_TARGETS run_camera_localization)

add_executable(run_image_slam run_image_slam.cc util/image_util.cc)
list(APPEND EXECUTABLE_TARGETS run_image_slam)

add_executable(run_image_localization run_image_localization.cc util/image_util.cc)
list(APPEND EXECUTABLE_TARGETS run_image_localization)

add_executable(run_video_slam run_video_slam.cc)
list(APPEND EXECUTABLE_TARGETS run_video_slam)

add_executable(run_video_localization run_video_localization.cc)
list(APPEND EXECUTABLE_TARGETS run_video_localization)

add_executable(run_euroc_slam run_euroc_slam.cc util/euroc_util.cc)
list(APPEND EXECUTABLE_TARGETS run_euroc_slam)

add_executable(run_kitti_slam run_kitti_slam.cc util/kitti_util.cc)
list(APPEND EXECUTABLE_TARGETS run_kitti_slam)

add_executable(run_tum_rgbd_slam run_tum_rgbd_slam.cc util/tum_rgbd_util.cc)
list(APPEND EXECUTABLE_TARGETS run_tum_rgbd_slam)

add_executable(run_camera_calibration run_camera_calibration.cc)
list(APPEND EXECUTABLE_TARGETS run_camera_calibration)

add_executable(run_video_calibration run_video_calibration.cc)
list(APPEND EXECUTABLE_TARGETS run_video_calibration)

# Additional Projects
# gps slam fusion
add_executable(gps_slam_fusion_video gps_slam_fusion_video.cc fusion/gps_fusion.cc fusion/gps_network.cc fusion/gps_parser.cc fusion/time_sync.cc)
list(APPEND EXECUTABLE_TARGETS gps_slam_fusion_video)
add_executable(gps_slam_fusion_camera gps_slam_fusion_camera.cc fusion/gps_fusion.cc fusion/gps_network.cc fusion/gps_parser.cc fusion/time_sync.cc)
list(APPEND EXECUTABLE_TARGETS gps_slam_fusion_camera)

target_link_libraries(gps_slam_fusion_video PRIVATE ${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-network.lib
	${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-system.lib)
target_link_libraries(gps_slam_fusion_camera PRIVATE ${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-network.lib
	${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-system.lib)

# include SFML headers for gps input
target_include_directories(gps_slam_fusion_video PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/SFML/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/example/fusion>)
target_include_directories(gps_slam_fusion_camera PRIVATE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/SFML/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/example/fusion>)


# gps video recording
add_executable(gps_video_rec gps_video_record.cc)
list(APPEND EXECUTABLE_TARGETS gps_video_rec)

target_link_libraries(gps_video_rec PRIVATE
                          ${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-network.lib
	${PROJECT_SOURCE_DIR}/3rd/SFML/lib/sfml-system.lib)

# include SFML headers for gps input
target_include_directories(gps_video_rec PRIVATE
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/SFML/include>)


foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # Set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # PangolinViewer is used on a priority basis
    if(USE_PANGOLIN_VIEWER)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_PANGOLIN_VIEWER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE pangolin_viewer)
    elseif(USE_SOCKET_PUBLISHER)
        # Set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_SOCKET_PUBLISHER)
        # Link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE socket_publisher)
    endif()

    # Setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${GLOG_LIBRARIES})
    endif()

    # Setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET} PRIVATE USE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE ${GPERFTOOLS_LIBRARIES})
    endif()

    # Link OpenVSLAM
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE
                          ${PROJECT_NAME}
                          opencv_imgcodecs
                          opencv_videoio)

    # include popl and spdlog headers
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
                               $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/spdlog/include>)
endforeach()
