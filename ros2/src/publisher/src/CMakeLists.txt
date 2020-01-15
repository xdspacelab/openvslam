set(EXECUTABLE_TARGETS "")

add_executable(video_publisher video_publisher.cc)
list(APPEND EXECUTABLE_TARGETS video_publisher)
set_target_properties(video_publisher PROPERTIES OUTPUT_NAME video PREFIX "")

add_executable(image_publisher image_publisher.cc ${OpenVSLAM_ROOT}/example/util/image_util.cc)
target_include_directories(image_publisher PRIVATE ${OpenVSLAM_ROOT}/example)
list(APPEND EXECUTABLE_TARGETS image_publisher)
set_target_properties(image_publisher PROPERTIES OUTPUT_NAME image PREFIX "")

foreach(EXECUTABLE_TARGET IN LISTS EXECUTABLE_TARGETS)
    ament_target_dependencies(${EXECUTABLE_TARGET}
      ${dependencies}
    )

    install(TARGETS
        ${EXECUTABLE_TARGET}
    DESTINATION lib/${PROJECT_NAME})
endforeach()
