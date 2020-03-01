# ----- Find dependencies -----

# Pangolin
find_package(Pangolin REQUIRED)

# ----- Configure PangolinViewer library -----

add_library(pangolin_viewer
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.h
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.h
            ${CMAKE_CURRENT_SOURCE_DIR}/viewer.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/color_scheme.cc)

set_target_properties(pangolin_viewer PROPERTIES
                      OUTPUT_NAME pangolin_viewer
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_include_directories(pangolin_viewer 
                           PUBLIC
                           ${Pangolin_INCLUDE_DIR})

target_link_libraries(pangolin_viewer
                      PUBLIC
                      ${PROJECT_NAME}
                      opencv_highgui
                      pangolin)

# ----- Install configuration -----

set(INSTALL_PANGOLIN_VIEWER OFF CACHE BOOL "Install PangolinViewer library")
if(INSTALL_PANGOLIN_VIEWER)
    set(PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/pangolin_viewer)

    install(TARGETS pangolin_viewer
            EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})

    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    install(FILES ${HEADERS}
            DESTINATION ${PANGOLIN_VIEWER_INCLUDE_INSTALL_DIR})
endif()
