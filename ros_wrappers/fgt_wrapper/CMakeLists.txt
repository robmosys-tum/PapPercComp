cmake_minimum_required(VERSION 3.0.2)
project(fgt_wrapper)
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS
        fgt/include
    LIBRARIES fgt
)
option(WITH_TESTS "" OFF)
add_subdirectory(fgt)