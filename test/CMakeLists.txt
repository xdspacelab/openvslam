set(INSTALL_GTEST OFF CACHE BOOL "Install google-test" FORCE)

# ----- Download google-test -----

include(${PROJECT_SOURCE_DIR}/cmake/DownloadProject.cmake)
download_project(PROJ googletest
                 URL https://github.com/google/googletest/archive/release-1.10.0.tar.gz
                 URL_HASH SHA1=9c89be7df9c5e8cb0bc20b3c4b39bf7e82686770
                 DOWNLOAD_NO_PROGRESS YES)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

# ----- Glob test codes----- 

file(GLOB_RECURSE OPENVSLAM_SOURCE_PATHS "./openvslam/*.cc")
list(APPEND SOURCE_PATHS ${OPENVSLAM_SOURCE_PATHS})

# ----- Build test executables -----

foreach(SOURCE_PATH ${SOURCE_PATHS})
    # Get relative path from ./test/
    file(RELATIVE_PATH SOURCE_REL_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCE_PATH})
    # Test module name: test_foo_bar
    string(REGEX REPLACE "\\.cc$" "" TEST_MODULE_NAME test/${SOURCE_REL_PATH})
    string(REPLACE "." "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    string(REPLACE "/" "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    # Executable name: test_foo_bar
    set(TEST_EXECUTABLE_NAME ${TEST_MODULE_NAME})

    # Create test executable
    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_PATH})
    target_compile_definitions(${TEST_EXECUTABLE_NAME} PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")
    target_include_directories(${TEST_EXECUTABLE_NAME} SYSTEM
                               PRIVATE
                               ${PROJECT_SOURCE_DIR}/src
                               ${googletest_SOURCE_DIR}/googletest/include)
    target_link_libraries(${TEST_EXECUTABLE_NAME}
                          PRIVATE
                          ${PROJECT_NAME}
                          test_helper
                          gtest_main
                          opencv_imgcodecs
                          opencv_highgui)
    set_target_properties(${TEST_EXECUTABLE_NAME} PROPERTIES
                          RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL ${PROJECT_BINARY_DIR}/test
                          RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO ${PROJECT_BINARY_DIR}/test)

    # Add test
    add_test(${TEST_MODULE_NAME} ${PROJECT_BINARY_DIR}/test/${TEST_EXECUTABLE_NAME})
endforeach()

# Add test helper library
add_subdirectory(helper)
