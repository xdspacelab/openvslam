file(GLOB_RECURSE CAMERA_SOURCES ./camera/*.cc)
file(GLOB_RECURSE CAMERA_HEADERS ./camera/*.h)

file(GLOB_RECURSE DATA_SOURCES ./data/*.cc)
file(GLOB_RECURSE DATA_HEADERS ./data/*.h)

file(GLOB_RECURSE FEATURE_SOURCES ./feature/*.cc)
file(GLOB_RECURSE FEATURE_HEADERS ./feature/*.h)

file(GLOB_RECURSE INITIALIZE_SOURCES ./initialize/*.cc)
file(GLOB_RECURSE INITIALIZE_HEADERS ./initialize/*.h)

file(GLOB_RECURSE IO_SOURCES ./io/*.cc)
file(GLOB_RECURSE IO_HEADERS ./io/*.h)

file(GLOB_RECURSE MATCH_SOURCES ./match/*.cc)
file(GLOB_RECURSE MATCH_HEADERS ./match/*.h)

file(GLOB_RECURSE MODULE_SOURCES ./module/*.cc)
file(GLOB_RECURSE MODULE_HEADERS ./module/*.h)

file(GLOB_RECURSE OPTIMIZE_SOURCES ./optimize/*.cc)
file(GLOB_RECURSE OPTIMIZE_HEADERS ./optimize/*.h)

file(GLOB_RECURSE PUBLISHER_SOURCES ./publisher/*.cc)
file(GLOB_RECURSE PUBLISHER_HEADERS ./publisher/*.h)

file(GLOB_RECURSE SOLVER_SOURCES ./solver/*.cc)
file(GLOB_RECURSE SOLVER_HEADERS ./solver/*.h)

file(GLOB_RECURSE UTIL_SOURCES ./util/*.cc)
file(GLOB_RECURSE UTIL_HEADERS ./util/*.h)

add_library(${PROJECT_NAME} SHARED
        type.h
        system.h system.cc
        config.h config.cc
        tracking_module.h tracking_module.cc
        mapping_module.h mapping_module.cc
        global_optimization_module.h global_optimization_module.cc
        ${CAMERA_SOURCES} ${CAMERA_HEADERS}
        ${DATA_SOURCES} ${DATA_HEADERS}
        ${FEATURE_SOURCES} ${FEATURE_HEADERS}
        ${INITIALIZE_SOURCES} ${INITIALIZE_HEADERS}
        ${IO_SOURCES} ${IO_HEADERS}
        ${MATCH_SOURCES} ${MATCH_HEADERS}
        ${MODULE_SOURCES} ${MODULE_HEADERS}
        ${OPTIMIZE_SOURCES} ${OPTIMIZE_HEADERS}
        ${PUBLISHER_SOURCES} ${PUBLISHER_HEADERS}
        ${SOLVER_SOURCES} ${SOLVER_HEADERS}
        ${UTIL_SOURCES} ${UTIL_HEADERS})

set_target_properties(${PROJECT_NAME} PROPERTIES
        OUTPUT_NAME openvslam
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${G2O_LIBS}
        ${LAPACK_LIBRARIES}
        ${CXSPARSE_LIBRARIES}
        ${SUITESPARSE_LIBRARIES}
        ${YAMLCPP_LIBRARIES}
        ${BOW_LIBS})
