add_subdirectory(g2o)

target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/pose_optimizer.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/local_bundle_adjuster.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/transform_optimizer.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/graph_optimizer.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/global_bundle_adjuster.cc)
