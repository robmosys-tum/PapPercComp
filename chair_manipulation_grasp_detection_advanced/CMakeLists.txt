cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_detection_advanced)

set(CMAKE_CXX_STANDARD 14)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
  pcl_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  moveit_core
)

find_package(PCL REQUIRED)
find_package(FCL REQUIRED)

catkin_package(CATKIN_DEPENDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

set(
  SOURCE_FILES
  include/chair_manipulation_grasp_detection_advanced/grasp.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_creator.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_element.h
  include/chair_manipulation_grasp_detection_advanced/grasp_sampler.h
  include/chair_manipulation_grasp_detection_advanced/model.h
  include/chair_manipulation_grasp_detection_advanced/utils.h
  include/chair_manipulation_grasp_detection_advanced/exception.h
  include/chair_manipulation_grasp_detection_advanced/wrench_space.h
  include/chair_manipulation_grasp_detection_advanced/gripper.h
  src/model.cpp
  src/utils.cpp
  src/grasp_database.cpp
  src/grasp_database_creator.cpp
  src/grasp_sampler.cpp
  src/gripper.cpp
)

set(
  LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
)

add_executable(create_grasp_database_node src/create_grasp_database_node.cpp ${SOURCE_FILES})
target_link_libraries(create_grasp_database_node ${LIBRARIES})

add_executable(grasp_detection_node src/grasp_detection_node.cpp ${SOURCE_FILES})
target_link_libraries(grasp_detection_node ${LIBRARIES})

add_executable(convert_mesh_test test/convert_mesh_test.cpp ${SOURCE_FILES})
target_link_libraries(convert_mesh_test ${LIBRARIES})

add_executable(grasp_sampler_test test/grasp_sampler_test.cpp ${SOURCE_FILES})
target_link_libraries(grasp_sampler_test ${LIBRARIES})

add_executable(grasp_test test/grasp_test.cpp ${SOURCE_FILES} include/chair_manipulation_grasp_detection_advanced/contact.h test/grasp_test.cpp)
target_link_libraries(grasp_test ${LIBRARIES})