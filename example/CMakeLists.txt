if(USE_PANGOLIN_VIEWER)
    message(STATUS "Viewer for examples: pangolin viewer")
elseif(USE_SOCKET_PUBLISHER)
    message(STATUS "Viewer for examples: socket publisher")
else()
    message(STATUS "Viewer for examples: none")
endif()

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

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # set output directory for executables
    set_target_properties(${EXECUTABLE_TARGET} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # pangolin_viewer is used on a priority basis
    if(USE_PANGOLIN_VIEWER)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_PANGOLIN_VIEWER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                pangolin_viewer)
    elseif(USE_SOCKET_PUBLISHER)
        # set macro flag
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_SOCKET_PUBLISHER)
        # link viewer
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                socket_publisher)
    endif()

    # setup stack trace logger
    if(USE_STACK_TRACE_LOGGER)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_STACK_TRACE_LOGGER)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GLOG_LIBRARIES})
    endif()

    # setup google-perftools
    if(USE_GOOGLE_PERFTOOLS)
        target_compile_definitions(${EXECUTABLE_TARGET}
                PRIVATE -DUSE_GOOGLE_PERFTOOLS)
        target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
                ${GPERFTOOLS_LIBRARIES})
    endif()

    # link OpenVSLAM
    target_link_libraries(${EXECUTABLE_TARGET} PRIVATE
            ${PROJECT_NAME})

    # include popl and spdlog headers
    target_include_directories(${EXECUTABLE_TARGET} PRIVATE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/spdlog/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/3rd/popl/include>
        )

endforeach()
