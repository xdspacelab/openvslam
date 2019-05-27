target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/initializer.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/relocalizer.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_tracker.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/keyframe_inserter.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/two_view_triangulator.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/local_map_cleaner.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/loop_detector.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/loop_bundle_adjuster.cc)
