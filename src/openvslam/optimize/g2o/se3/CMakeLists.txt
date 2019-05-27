target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex_container.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/perspective_pose_opt_edge.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/perspective_reproj_edge.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular_pose_opt_edge.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular_reproj_edge.cc)
