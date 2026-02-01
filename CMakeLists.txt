cmake_minimum_required(VERSION 3.29)
project(Intersect3D)

set(CMAKE_CXX_STANDARD 20)

add_executable(Intersect3D Segment3DTest.cpp
        Segment3D.cpp)