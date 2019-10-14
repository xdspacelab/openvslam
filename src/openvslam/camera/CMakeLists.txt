target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/base.h
        ${CMAKE_CURRENT_SOURCE_DIR}/perspective.h
        ${CMAKE_CURRENT_SOURCE_DIR}/fisheye.h
        ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular.h
        ${CMAKE_CURRENT_SOURCE_DIR}/base.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/perspective.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/fisheye.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/equirectangular.cc)

# Install public header
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/camera)
