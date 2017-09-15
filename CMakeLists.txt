cmake_minimum_required(VERSION 2.8)
project(pso)

find_package(eigen3)

set(PMR_INCLUDE_DIRS src)
include_directories("/usr/include/eigen3")

set(SRC_LIST 
src/particle.cpp 
src/pose.cpp
src/transform.cpp
src/swarm.cpp
src/test.cpp
)

add_executable(test ${SRC_LIST})

