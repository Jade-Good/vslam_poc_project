message(STATUS "Finding yaml-cpp...")

find_package(yaml-cpp REQUIRED)

if(yaml-cpp_FOUND)
  message(STATUS "Found yaml-cpp - ${YAML_CPP_INCLUDE_DIR}")

  include_directories(${YAML_CPP_INCLUDE_DIR})

  set(YAML_CPP_LIBS ${YAML_CPP_LIBRARIES})

endif(yaml-cpp_FOUND)

# find_library(YAML_CPP_LIB NAMES yaml-cpp PATHS ${SEARCH_LIBS})
# find_path(YAML_CPP_INCLUDE NAMES yaml-cpp/yaml.h PATHS ${SEARCH_HEADERS})
# if (YAML_CPP_INCLUDE)
#   message("-- Found yaml-cpp: " ${YAML_CPP_INCLUDE})
# endif (YAML_CPP_INCLUDE)