add_library(${PROJECT_NAME} SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/config.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/system.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/tracking_module.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/mapping_module.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/global_optimization_module.cc)

add_subdirectory(camera)
add_subdirectory(data)
add_subdirectory(feature)
add_subdirectory(initialize)
add_subdirectory(io)
add_subdirectory(match)
add_subdirectory(module)
add_subdirectory(optimize)
add_subdirectory(publish)
add_subdirectory(solve)
add_subdirectory(util)

target_sources(${PROJECT_NAME} PRIVATE)

set_target_properties(${PROJECT_NAME} PROPERTIES
        OUTPUT_NAME openvslam
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(${PROJECT_NAME}
        Threads::Threads
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        ${LAPACK_LIBRARIES}
        ${CXSPARSE_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${BOW_LIBS})
