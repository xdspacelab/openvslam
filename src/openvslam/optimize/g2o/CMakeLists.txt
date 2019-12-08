add_subdirectory(se3)
add_subdirectory(sim3)

target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex.h
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex_container.h
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex_container.cc)

# Install headers
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/optimize/g2o)
