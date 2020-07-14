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
  moveit_visual_tools
  qhull_ros
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
  include/chair_manipulation_grasp_detection_advanced/grasp_hypothesis.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_creator.h
  include/chair_manipulation_grasp_detection_advanced/grasp_database_element.h
  include/chair_manipulation_grasp_detection_advanced/grasp_sampler.h
  include/chair_manipulation_grasp_detection_advanced/model.h
  include/chair_manipulation_grasp_detection_advanced/utils.h
  include/chair_manipulation_grasp_detection_advanced/exception.h
  include/chair_manipulation_grasp_detection_advanced/wrench.h
  include/chair_manipulation_grasp_detection_advanced/gripper.h
  include/chair_manipulation_grasp_detection_advanced/transform.h
  include/chair_manipulation_grasp_detection_advanced/contact.h
  include/chair_manipulation_grasp_detection_advanced/multi_arm_grasp.h
  include/chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h
  include/chair_manipulation_grasp_detection_advanced/qhull_mutex.h
  include/chair_manipulation_grasp_detection_advanced/stopwatch.h
  src/model.cpp
  src/utils.cpp
  src/grasp_database.cpp
  src/grasp_database_creator.cpp
  src/grasp_sampler.cpp
  src/gripper.cpp
  src/wrench.cpp
  src/transform.cpp
  src/grasp_synthesizer.cpp
  src/qhull_mutex.cpp
)

set(
  TEST_SOURCE_FILES
  test/test_common.h
  test/test_common.cpp
)

set(
  LIBRARIES
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${FCL_LIBRARIES}
)

# Nodes

add_executable(create_grasp_database_node src/create_grasp_database_node.cpp ${SOURCE_FILES})
target_link_libraries(create_grasp_database_node ${LIBRARIES})

add_executable(grasp_detection_node src/grasp_detection_node.cpp ${SOURCE_FILES})
target_link_libraries(grasp_detection_node ${LIBRARIES})

# Tests

add_executable(convert_mesh_test test/convert_mesh_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(convert_mesh_test ${LIBRARIES})

add_executable(grasp_sampler_test test/grasp_sampler_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(grasp_sampler_test ${LIBRARIES})

add_executable(gripper_test test/gripper_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(gripper_test ${LIBRARIES})

add_executable(wrench_test test/wrench_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(wrench_test ${LIBRARIES})

add_executable(load_database_test test/load_database_test.cpp ${SOURCE_FILES} ${TEST_SOURCE_FILES})
target_link_libraries(load_database_test ${LIBRARIES})