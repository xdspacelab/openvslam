# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.h
               ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/publish)
