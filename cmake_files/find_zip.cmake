message(STATUS "Finding Zip...")

find_package(zip)
# find_library(ZIP_LIB NAMES libzip zip PATHS ${SEARCH_LIBS})
# find_path(ZIP_INCLUDE NAMES zip.h PATHS ${SEARCH_HEADERS})
# if (ZIP_INCLUDE)
#   message("-- Found libzip: " ${ZIP_INCLUDE})
# endif (ZIP_INCLUDE)