target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/common.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/frame.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/keyframe.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/landmark.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/graph_node.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/camera_database.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/map_database.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/bow_database.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_statistics.cc)
