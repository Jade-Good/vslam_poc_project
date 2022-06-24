cmake_minimum_required(VERSION 3.22)

find_package(GTest CONFIG REQUIRED HINTS ${CMAKE_SOURCE_DIR}/thirdparty/googletest/install/lib/cmake/GTest)

if(GTest_FOUND)
    set(GTEST_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/thirdparty/googletest/install/include)
    message(STATUS "Found GTest - ${GTEST_INCLUDE_DIRS}")

    include_directories(${CMAKE_SOURCE_DIR}/thirdparty/googletest/install/include)
    set(GTEST_LIBS GTest::gtest)
endif(GTest_FOUND)


# Test sources
set(MODULE_TEST_SRCS
        Test
        )

# Include
include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${Pangolin_INCLUDE_DIRS}
)


# Targets
add_executable(unit_tests
        main.cpp
        ${MODULE_TEST_SRCS}.cpp
        )

target_link_libraries(unit_tests
        ${GTEST_LIBS}
        ORB_SLAM2
        stdc++fs
        )

add_test(NAME unit_tests COMMAND
        ${CMAKE_CURRENT_BINARY_DIR}/unit_tests)


set(matchString "/")
set(replaceWith "_")

foreach(test ${MODULE_TEST_SRCS})
    string(REPLACE ${matchString} ${replaceWith} test_name ${test})
    set(test_name ${test_name})

    add_executable(${test_name}
            main.cpp
            ${test}.cpp)
    target_link_libraries(${test_name}
            ${GTEST_LIBS}
            ORB_SLAM2
            stdc++fs
            )
    add_test(${test_name}
            ${test_name}
            --gtest_color=yes
            )
endforeach()