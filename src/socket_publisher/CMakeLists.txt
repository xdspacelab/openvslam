protobuf_generate_cpp(MAP_PB_SOURCE MAP_PB_HEADER protobuf/map_segment.proto)
set_source_files_properties(${MAP_PB_HEADER} ${MAP_PB_SOURCE} COMPILE_FLAGS -Wno-unused-variable)

add_library(socket_publisher SHARED
        publisher.cc
        socket_client.cc
        data_serializer.cc
        ${MAP_PB_SOURCE})

set_target_properties(socket_publisher PROPERTIES
        OUTPUT_NAME socket_publisher
        LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

# include for protobuf scheme
target_include_directories(socket_publisher PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR})

target_include_directories(socket_publisher PUBLIC
        ${SIOCLIENT_INCLUDE_DIR}
        ${PROTOBUF_INCLUDE_DIRS})

target_link_libraries(socket_publisher
        ${PROJECT_NAME}
        ${SIOCLIENT_LIBRARY}
        ${PROTOBUF_LIBRARIES})

install(TARGETS socket_publisher
      EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
      RUNTIME DESTINATION ${RUNTIME_DESTINATION}
      LIBRARY DESTINATION ${LIBRARY_DESTINATION}
      ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
      INCLUDES DESTINATION ${SOCKER_VIEWER_INCLUDE_INSTALL_DIR}
    )

file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${headers} DESTINATION ${SOCKER_VIEWER_INCLUDE_INSTALL_DIR})
