# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular_pose_opt_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular_reproj_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/perspective_pose_opt_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/perspective_reproj_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/pose_opt_edge_wrapper.h
               ${CMAKE_CURRENT_SOURCE_DIR}/reproj_edge_wrapper.h
               ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex_container.h
               ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex.h)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/optimize/internal/se3)
