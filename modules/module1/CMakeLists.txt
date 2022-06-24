################################################################################
# Project Name: module1
#
# Components:
#    - ExampleComponent: Wrapper class of ExampleLibrary
#
################################################################################

# Project information
cmake_minimum_required(VERSION 3.10)
set(PACKAGE_NAME module1)
project(${PACKAGE_NAME} LANGUAGES CXX)

# Export aliases
set(MODULE1_PATH ${CMAKE_CURRENT_SOURCE_DIR} CACHE PATH "MODULE1_PATH")
set(MODULE1_INCLUDE_PATH
        "${MODULE1_PATH}/include" CACHE PATH "MODULE1_INCLUDE_PATH")
set(MODULE1_RESOURCES_PATH
        "${MODULE1_PATH}/resources" CACHE PATH "MODULE1_RESOURCES_PATH")
set(MODULE1_TEST_PATH
        "${MODULE1_PATH}/test" CACHE PATH "MODULE1_TEST_PATH")

# Paths
add_definitions(-DMODULE1_SOURCE_DIR="${MODULE1_PATH}")
add_definitions(-DMODULE1_RESOURCES_DIR="${MODULE1_RESOURCES_PATH}")

# Source files
set(MODULE1_SOURCE_FILES
        src/Class.cpp
        )
add_library(${PACKAGE_NAME}
        ${MODULE1_SOURCE_FILES}
        )

# Header files, including 3rdParty
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

target_include_directories(${PACKAGE_NAME} PRIVATE
        ${MODULE1_INCLUDE_PATH}
        )
target_link_libraries(${PACKAGE_NAME} PRIVATE
        ${EASY_PROFILER_LIBS}
#        ${SPDLOG_LIBS}
        stdc++fs
        )

# Compile options
target_compile_options(${PACKAGE_NAME} PRIVATE
        -Wall -Wextra -Wpedantic -Werror
        )
set_target_properties(${PACKAGE_NAME} PROPERTIES
        CXX_STANDARD 11
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        )

# Unit tests
add_subdirectory(test)
