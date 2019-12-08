# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex.h
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_opt_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/transform_vertex.h
               ${CMAKE_CURRENT_SOURCE_DIR}/backward_reproj_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/forward_reproj_edge.h
               ${CMAKE_CURRENT_SOURCE_DIR}/shot_vertex.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_opt_edge.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/transform_vertex.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/backward_reproj_edge.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/forward_reproj_edge.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/optimize/g2o/sim3)
