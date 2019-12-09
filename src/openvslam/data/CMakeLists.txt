# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/bow_vocabulary.h
               ${CMAKE_CURRENT_SOURCE_DIR}/common.h
               ${CMAKE_CURRENT_SOURCE_DIR}/frame.h
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe.h
               ${CMAKE_CURRENT_SOURCE_DIR}/landmark.h
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_node.h
               ${CMAKE_CURRENT_SOURCE_DIR}/camera_database.h
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database.h
               ${CMAKE_CURRENT_SOURCE_DIR}/bow_database.h
               ${CMAKE_CURRENT_SOURCE_DIR}/frame_statistics.h
               ${CMAKE_CURRENT_SOURCE_DIR}/common.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/frame.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/keyframe.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/landmark.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/graph_node.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/camera_database.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/map_database.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/bow_database.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/frame_statistics.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
        DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/data)
