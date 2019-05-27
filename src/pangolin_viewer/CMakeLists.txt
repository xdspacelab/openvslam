add_library(pangolin_viewer SHARED
        viewer.cc
        color_scheme.cc)

set_target_properties(pangolin_viewer PROPERTIES
        OUTPUT_NAME pangolin_viewer
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_include_directories(pangolin_viewer PUBLIC
        ${Pangolin_INCLUDE_DIRS})

target_link_libraries(pangolin_viewer
        ${PROJECT_NAME}
        ${Pangolin_LIBRARIES})
