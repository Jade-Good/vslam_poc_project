# Test sources
set(MODULE1_TEST_SRCS
        sample1
        )

# Include
include_directories(
        ${MODULE1_INCLUDE_PATH}
)

# Targets
add_executable(unit_tests
        main.cpp
        ${MODULE1_TEST_SRCS}.cpp
        )

target_link_libraries(unit_tests
        ${GTEST_LIBS}
        module1
        stdc++fs
        )

add_test(NAME UnitTest COMMAND
        ${CMAKE_CURRENT_BINARY_DIR}/unit_tests)

set(matchString "/")
set(replaceWith "_")

foreach(test ${MODULE1_TEST_SRCS})
    string(REPLACE ${matchString} ${replaceWith} test_name ${test})
    set(test_name ${test_name})

    add_executable(${test_name}
            main.cpp
            ${test}.cpp)
    target_link_libraries(${test_name}
            ${GTEST_LIBS}
            module1
            stdc++fs
            )
    add_test(${test_name}
            ${test_name}
            --gtest_color=yes
            )
endforeach()