message(STATUS "Finding GLUT,GLEW,OpenGL...")

if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.11)
    cmake_policy(SET CMP0072 NEW) # Prefer GLVND over Legacy OpenGL
endif()

if (APPLE)
    set(CMAKE_EXE_LINKER_FLAGS "-framework OpenGL -framework GLUT")
else (APPLE)  # Linux, not so much
    find_package(OpenGL REQUIRED)
    find_package(GLUT REQUIRED)
    include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})
endif (APPLE)

find_package(GLEW)
if (${GLEW_FOUND})
    message("-- Found optional glew: " ${GLEW_INCLUDE_DIRS})
    include_directories(${GLEW_INCLUDE_DIRS})
endif (${GLEW_FOUND})

message(STATUS "Finding GLFW,GLM...")

find_package(glfw3 REQUIRED)
if (${glfw3_FOUND})
    message("-- Found optional glfw: " ${glfw3_INCLUDE_DIR})
    include_directories(${glfw3_INCLUDE_DIR})
endif (${glfw3_FOUND})

find_package(glm REQUIRED)
if (${glm_FOUND})
    message("-- Found optional glm: " ${GLM_INCLUDE_DIRS})
    include_directories(${GLM_INCLUDE_DIRS})
endif (${glm_FOUND})

set(GL_LIBS ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} glfw ${GLM_LIBRARIES})