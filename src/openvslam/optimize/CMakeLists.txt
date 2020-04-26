# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/pose_optimizer.h
               ${CMAKE_CURRENT_SOURCE_DIR}/local_bundle_adjuster.h
               ${CMAKE_CURRENT_SOURCE_DIR}/transform_optimizer.h
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_optimizer.h
               ${CMAKE_CURRENT_SOURCE_DIR}/global_bundle_adjuster.h
               ${CMAKE_CURRENT_SOURCE_DIR}/pose_optimizer.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/local_bundle_adjuster.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/transform_optimizer.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_optimizer.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/global_bundle_adjuster.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/optimize)

# Append subdirectory
add_subdirectory(internal)
