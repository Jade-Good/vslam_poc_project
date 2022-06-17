message(STATUS "Finding PNG,JPEG,TIFF,ZLIB...")

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(${PNG_INCLUDE_DIR})
include_directories(${JPEG_INCLUDE_DIR})
include_directories(${TIFF_INCLUDE_DIR})
include_directories(${ZLIB_INCLUDE_DIR})

set(IMAGE_LIBS ${PNG_LIBRARY} 
    ${JPEG_LIBRARY} 
    ${TIFF_LIBRARY} 
    ${ZLIB_LIBRARY})