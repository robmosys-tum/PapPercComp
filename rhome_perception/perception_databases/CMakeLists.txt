cmake_minimum_required(VERSION 3.5)
project(perception_databases)

# Default to C++14


find_package(ament_cmake REQUIRED)



ament_export_dependencies(${PROJECT_NAME})

install(DIRECTORY manipulable_objects
  DESTINATION share/${PROJECT_NAME})
#   FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")

ament_package()
