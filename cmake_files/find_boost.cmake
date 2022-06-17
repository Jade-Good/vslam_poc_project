message(STATUS "Finding Boost...")

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    find_package(Boost COMPONENTS filesystem REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/boost/install/Debug/lib/cmake/Boost-1.78.0)
endif (CMAKE_BUILD_TYPE MATCHES "Debug")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    find_package(Boost COMPONENTS filesystem REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/boost/install/Release/lib/cmake/Boost-1.78.0)
endif (CMAKE_BUILD_TYPE MATCHES "Release")

if (Boost_FOUND)
    message(STATUS "Found Boost library: " ${Boost_INCLUDE_DIRS})

    include_directories(${Boost_INCLUDE_DIRS})
endif (Boost_FOUND)
