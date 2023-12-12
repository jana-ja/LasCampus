cmake_minimum_required(VERSION 3.26)
project(LasCampus)

set(CMAKE_CXX_STANDARD 17)

# added -fopenmp
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp:llvm")

add_executable(LasCampus src/main.cpp
        src/DataStructure.cpp
        src/DataStructure.h
        src/Window.cpp
        src/Window.h
        src/Camera.h
        src/shader.h
        src/util.h
        src/LasDataIO.h
        src/LasDataIO.cpp
        src/ShpDataIO.cpp
        src/ShpDataIO.h
        src/UTM.h
        src/UTM.cpp
)

if (WIN32)
    # glfw and glew from source
    set(GLFW_SOURCE_DIR "third-party/glfw-3.3.8")
    set(GLEW_SOURCE_DIR "third-party/glew-2.1.0")

    set(BUILD_SHARED_LIBS OFF CACHE BOOL "")
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "")
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "")
    set(GLFW_BUILD_DOCS OFF CACHE BOOL "")
    set(GLFW_INSTALL OFF CACHE BOOL "")
    add_subdirectory(${GLFW_SOURCE_DIR} ${GLEW_SOURCE_DIR})
    add_definitions(-DGLEW_STATIC)
    add_library(glew STATIC ${GLEW_SOURCE_DIR}/src/glew.c ${GLEW_SOURCE_DIR}/include)
    set_property(TARGET glew PROPERTY POSITION_INDEPENDENT_CODE ON)
    target_include_directories(glew PUBLIC ${GLEW_SOURCE_DIR}/include ${GLFW_SOURCE_DIR}/include)
    target_link_libraries(glew PUBLIC ${GLFW_LIBRARIES})

    # openGL
    find_package(OpenGL REQUIRED)
    target_link_libraries(LasCampus PUBLIC ${OPENGL_LIBRARIES})

    target_link_libraries(LasCampus PUBLIC glew glfw)

    # GLM
    set(GLM_SOURCE_DIR "third-party/glm")
    add_subdirectory(${GLM_SOURCE_DIR})
    target_link_libraries(LasCampus PUBLIC glm::glm)

    # OpenMP
    find_package(OpenMP REQUIRED)
    target_link_libraries(LasCampus PRIVATE OpenMP::OpenMP_CXX)

    # PCL
    find_package(PCL CONFIG REQUIRED)
    target_link_libraries(LasCampus PRIVATE ${PCL_LIBRARIES})

elseif (APPLE)
    # from mac:
    find_package(OpenGL REQUIRED)
    include_directories(${OPENGL_INCLUDE_DIR})
    target_link_libraries(LasCampus ${OPENGL_LIBRARIES})

    find_package(GLEW REQUIRED)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(LasCampus GLEW::GLEW)#${GLEW_LIBRARY})#GLEW::GLEW)#${GLEW_LIBRARIES}) #GLEW::glew)

    find_package(glfw3 3.3 REQUIRED)
    target_link_libraries(LasCampus glfw)

    find_package(GLM REQUIRED)
    include_directories(${GLM_INCLUDE_DIRS})
    target_link_libraries(LasCampus ${GLM_LIBRARY})

    # PCL
    find_package(PCL CONFIG REQUIRED)
    target_link_libraries(LasCampus ${PCL_LIBRARIES})

endif ()
