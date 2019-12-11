# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/converter.h
               ${CMAKE_CURRENT_SOURCE_DIR}/image_converter.h
               ${CMAKE_CURRENT_SOURCE_DIR}/random_array.h
               ${CMAKE_CURRENT_SOURCE_DIR}/converter.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/image_converter.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/random_array.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo_rectifier.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/util)
