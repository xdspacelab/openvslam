target_sources(${PROJECT_NAME} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.h
        ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.h
        ${CMAKE_CURRENT_SOURCE_DIR}/frame_publisher.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/map_publisher.cc)

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/publish)
