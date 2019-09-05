# download google-test
include(${PROJECT_SOURCE_DIR}/cmake/DownloadProject.cmake)
download_project(PROJ googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG master
        UPDATE_DISCONNECTED 1)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})

# glob test codes
file(GLOB_RECURSE OPENVSLAM_SOURCE_PATHS ./openvslam/*.cc)
list(APPEND SOURCE_PATHS ${OPENVSLAM_SOURCE_PATHS})

# build test executables
foreach(SOURCE_PATH ${SOURCE_PATHS})

    # get relative path from ./test/
    file(RELATIVE_PATH SOURCE_REL_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${SOURCE_PATH})
    # test module name: test_foo_bar
    string(REGEX REPLACE "\\.cc$" "" TEST_MODULE_NAME test/${SOURCE_REL_PATH})
    string(REPLACE "." "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    string(REPLACE "/" "_" TEST_MODULE_NAME ${TEST_MODULE_NAME})
    # executable name: test_foo_bar
    set(TEST_EXECUTABLE_NAME ${TEST_MODULE_NAME})

    # create test executable
    add_executable(${TEST_EXECUTABLE_NAME} ${SOURCE_PATH})
    target_compile_definitions(${TEST_EXECUTABLE_NAME}
            PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data/")
    target_include_directories(${TEST_EXECUTABLE_NAME} SYSTEM
            PRIVATE ${PROJECT_SOURCE_DIR}/src
            PRIVATE ${googletest_SOURCE_DIR}/googletest/include)
    target_link_libraries(${TEST_EXECUTABLE_NAME}
            ${PROJECT_NAME}
            test_helper
            gtest_main)

    set_target_properties(${TEST_EXECUTABLE_NAME} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}"
            RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}")

    # add test
    add_test(${TEST_MODULE_NAME} ${PROJECT_BINARY_DIR}/${TEST_EXECUTABLE_NAME})
endforeach()

# add test helper library
add_subdirectory(helper)
