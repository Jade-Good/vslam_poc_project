message(STATUS "Finding Cereal...")

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    find_package(cereal REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/install/Debug/lib/cmake/cereal)
    set(cereal_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/install/Debug/include)
endif (CMAKE_BUILD_TYPE MATCHES "Debug")

if (CMAKE_BUILD_TYPE MATCHES "Release")
    find_package(cereal REQUIRED HINTS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/install/Release/lib/cmake/cereal)
    set(cereal_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/cereal/install/Release/include)
endif (CMAKE_BUILD_TYPE MATCHES "Release")

if (cereal_FOUND)
    message(STATUS "Found cereal library - ${cereal_INCLUDE_DIRS}")

    include_directories(${cereal_INCLUDE_DIRS})

    set(CEREAL_LIBS cereal::cereal)
endif (cereal_FOUND)
