add_subdirectory(se3)
add_subdirectory(sim3)

target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark_vertex_container.cc)
