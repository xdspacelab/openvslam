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
