cmake_minimum_required(VERSION 3.26)
project(Masti)

set(CMAKE_CXX_STANDARD 17)

add_executable(Masti src/main.cpp
        src/PointCloud.cpp
        src/PointCloud.h
        src/DummyData.cpp
        src/DummyData.h
        src/Vertex.h
        src/Window.cpp
        src/Window.h
        src/loadShader.cpp
        src/loadShader.h)

find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})
target_link_libraries(Masti ${OPENGL_LIBRARIES})

find_package(GLEW REQUIRED)
include_directories(${GLEW_INCLUDE_DIRS})
target_link_libraries(Masti GLEW::GLEW)#${GLEW_LIBRARY})#GLEW::GLEW)#${GLEW_LIBRARIES}) #GLEW::glew)

find_package(glfw3 3.3 REQUIRED)
target_link_libraries(Masti glfw)



# installier den shit, dann hier linken undso, da stand nochmal was zu opengl und extension loader library wtf