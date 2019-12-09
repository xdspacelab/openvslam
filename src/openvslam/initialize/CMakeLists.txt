# Add sources
target_sources(${PROJECT_NAME}
               PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/base.h
               ${CMAKE_CURRENT_SOURCE_DIR}/perspective.h
               ${CMAKE_CURRENT_SOURCE_DIR}/bearing_vector.h
               ${CMAKE_CURRENT_SOURCE_DIR}/base.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/perspective.cc
               ${CMAKE_CURRENT_SOURCE_DIR}/bearing_vector.cc)

# Install headers
file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${HEADERS}
       DESTINATION ${OPENVSLAM_INCLUDE_INSTALL_DIR}/initialize)
