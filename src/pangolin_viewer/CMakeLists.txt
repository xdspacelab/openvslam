add_library(pangolin_viewer SHARED
        viewer.cc
        viewer.h
        color_scheme.cc
        color_scheme.h)

set_target_properties(pangolin_viewer PROPERTIES
        OUTPUT_NAME pangolin_viewer
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(pangolin_viewer PUBLIC
        ${PROJECT_NAME} 
        pangolin)

install(TARGETS pangolin_viewer
      EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
      RUNTIME DESTINATION ${RUNTIME_DESTINATION}
      LIBRARY DESTINATION ${LIBRARY_DESTINATION}
      ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
      INCLUDES DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR}
    )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})
