# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex_container.h
               ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex.h)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/optimize/internal)

# Append subdirectory
add_subdirectory(se3)
add_subdirectory(sim3)
