cmake_minimum_required(VERSION 3.26)
project(Masti)

set(CMAKE_CXX_STANDARD 17)

add_executable(Masti src/main.cpp
        src/PointCloud.cpp
        src/PointCloud.h
        src/getDummyData.cpp
        src/getDummyData.h
        src/Vertex.h
        src/Window.cpp
        src/Window.h
        src/loadShader.cpp
        src/loadShader.h)

#find_package(OpenGL REQUIRED)
#include_directories(${OPENGL_INCLUDE_DIR})
#target_link_libraries(Masti ${OPENGL_LIBRARIES})

#set(GLEW_DIR "third-party/glew-2.1.0")
#find_package(GLEW REQUIRED)
#include_directories(${GLEW_INCLUDE_DIRS})
#link_libraries(${GLEW_LIBRARIES})


#set(GLFW_SOURCE_DIR "third-party/glfw-3.3.8")
#set(GLEW_SOURCE_DIR "third-party/glew-2.1.0")
#
#set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
#set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
#set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
#set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
#set(GLFW_INSTALL OFF CACHE BOOL "")
#add_subdirectory(${GLFW_SOURCE_DIR} ${GLEW_SOURCE_DIR})
#add_definitions(-DGLEW_STATIC)
#add_library(glew STATIC ${GLEW_SOURCE_DIR}/src/glew.c ${GLEW_SOURCE_DIR}/include)
#set_property(TARGET glew PROPERTY POSITION_INDEPENDENT_CODE ON)
#target_include_directories(glew PUBLIC ${GLEW_SOURCE_DIR}/include ${GLFW_SOURCE_DIR}/include)
#target_link_libraries(glew PUBLIC ${GLFW_LIBRARIES})


#target_link_libraries(Masti glew)
#find_package(GLEW REQUIRED)
##
#include_directories(${GLEW_INCLUDE_DIRS})
#target_link_libraries(Masti GLEW::GLEW)#${GLEW_LIBRARY})#GLEW::GLEW)#${GLEW_LIBRARIES}) #GLEW::glew)
#
#find_package(glfw3 3.3 REQUIRED)
#target_link_libraries(Masti glfw) # (Masti "C:\\Program Files\\JetBrains\\CLion 2023.2.1\\bin\\mingw\\lib\\libglfw3dll.a")



# from mac:
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
target_link_libraries(Masti ${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
target_link_libraries(Masti GLEW::GLEW)#${GLEW_LIBRARY})#GLEW::GLEW)#${GLEW_LIBRARIES}) #GLEW::glew)

find_package(glfw3 3.3 REQUIRED)
target_link_libraries(Masti glfw)

find_package(GLM REQUIRED)
include_directories(${GLM_INCLUDE_DIRS})
target_link_libraries(Masti ${GLM_LIBRARY})

