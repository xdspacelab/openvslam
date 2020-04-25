# ----- Find dependencies -----

# socket.io-cpp-client
find_package(sioclient REQUIRED)

# Protobuf
find_package(Protobuf REQUIRED)
if(NOT PROTOBUF_PROTOC_EXECUTABLE)
    message(FATAL_ERROR "Could not find protoc executable (PROTOBUF_PROTOC_EXECUTABLE)")
endif()
message(STATUS "Found protoc executable: ${PROTOBUF_PROTOC_EXECUTABLE}")

# ----- Protobuf transpile -----

protobuf_generate_cpp(MAP_PB_SOURCE MAP_PB_HEADER protobuf/map_segment.proto)
if(NOT MSVC)
    set_source_files_properties(${MAP_PB_HEADER} ${MAP_PB_SOURCE}
                                COMPILE_FLAGS -Wno-unused-variable)
endif()

# ----- Configure SocketPublisher library -----

add_library(socket_publisher
            ${CMAKE_CURRENT_SOURCE_DIR}/data_serializer.h
            ${CMAKE_CURRENT_SOURCE_DIR}/publisher.h
            ${CMAKE_CURRENT_SOURCE_DIR}/socket_client.h
            ${CMAKE_CURRENT_SOURCE_DIR}/data_serializer.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/publisher.cc
            ${CMAKE_CURRENT_SOURCE_DIR}/socket_client.cc
            ${MAP_PB_SOURCE})

set_target_properties(socket_publisher PROPERTIES
                      OUTPUT_NAME socket_publisher
                      ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)

target_include_directories(socket_publisher
                           PUBLIC
                           ${SIOCLIENT_INCLUDE_DIR}
                           ${PROTOBUF_INCLUDE_DIRS}
                           PRIVATE
                           ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(socket_publisher
                      PUBLIC
                      ${PROJECT_NAME}
                      opencv_imgcodecs
                      ${SIOCLIENT_LIBRARY}
                      ${PROTOBUF_LIBRARIES})

# ----- Install configuration -----

set(INSTALL_SOCKET_PUBLISHER OFF CACHE BOOL "Install SocketPublisher library")
if(INSTALL_SOCKET_PUBLISHER)
    set(SOCKER_PUBLISHER_INCLUDE_INSTALL_DIR ${INCLUDES_DESTINATION}/socket_publisher)

    install(TARGETS socket_publisher
            EXPORT ${OPENVSLAM_TARGETS_EXPORT_NAME}
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${SOCKER_PUBLISHER_INCLUDE_INSTALL_DIR})

    file(GLOB HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
    install(FILES ${HEADERS}
            DESTINATION ${SOCKER_PUBLISHER_INCLUDE_INSTALL_DIR})
endif()
