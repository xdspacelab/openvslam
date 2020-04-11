set(EXECUTABLE_TARGETS "")

add_executable(video_publisher video_publisher.cc)
list(APPEND EXECUTABLE_TARGETS video_publisher)
set_target_properties(video_publisher PROPERTIES OUTPUT_NAME video PREFIX "")

add_executable(image_publisher image_publisher.cc ${OpenVSLAM_ROOT}/example/util/image_util.cc)
list(APPEND EXECUTABLE_TARGETS image_publisher)
target_include_directories(image_publisher PRIVATE ${OpenVSLAM_ROOT}/example)
set_target_properties(image_publisher PROPERTIES OUTPUT_NAME image PREFIX "")

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    # build
    target_include_directories(${EXECUTABLE_TARGET}
                               PRIVATE rclcpp_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${EXECUTABLE_TARGET}
                          PRIVATE ${rclcpp_LIBRARIES} ${OpenCV_LIBRARIES})
    ament_target_dependencies(${EXECUTABLE_TARGET}
                              PUBLIC rclcpp cv_bridge image_transport rcutils)
    install(TARGETS ${EXECUTABLE_TARGET}
            DESTINATION lib/${PROJECT_NAME})
endforeach()
