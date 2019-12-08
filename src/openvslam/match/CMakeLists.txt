# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/area.h
               ${CMAKE_CURRENT_SOURCE_DIR}/bow_tree.h
               ${CMAKE_CURRENT_SOURCE_DIR}/fuse.h
               ${CMAKE_CURRENT_SOURCE_DIR}/projection.h
               ${CMAKE_CURRENT_SOURCE_DIR}/robust.h
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo.h
               ${CMAKE_CURRENT_SOURCE_DIR}/area.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/bow_tree.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/fuse.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/projection.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/robust.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/stereo.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/match)
