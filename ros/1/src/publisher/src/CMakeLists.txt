set(EXECUTABLE_TARGETS "")

add_executable(video_publisher video_publisher.cc)
list(APPEND EXECUTABLE_TARGETS video_publisher)
set_target_properties(video_publisher PROPERTIES OUTPUT_NAME video PREFIX "")

add_executable(image_publisher image_publisher.cc ${OpenVSLAM_ROOT}/example/util/image_util.cc)
target_include_directories(image_publisher PRIVATE ${OpenVSLAM_ROOT}/example)
list(APPEND EXECUTABLE_TARGETS image_publisher)
set_target_properties(image_publisher PROPERTIES OUTPUT_NAME image PREFIX "")

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    add_dependencies(${EXECUTABLE_TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()
