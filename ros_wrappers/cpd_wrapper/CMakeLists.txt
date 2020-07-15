cmake_minimum_required(VERSION 3.0.2)
project(cpd_wrapper)
find_package(catkin REQUIRED fgt_wrapper)
catkin_package(
    INCLUDE_DIRS
        cpd/include
    LIBRARIES cpd
)
option(WITH_TESTS "Build test suite" OFF)
add_subdirectory(cpd)