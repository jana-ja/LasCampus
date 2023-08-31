cmake_minimum_required(VERSION 3.26)
project(Masti)

set(CMAKE_CXX_STANDARD 17)

add_executable(Masti src/main.cpp
        src/PointCloud.cpp
        src/PointCloud.h)
