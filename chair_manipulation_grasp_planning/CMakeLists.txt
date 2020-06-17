cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_planning)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    pluginlib
    geometric_shapes
    pcl_ros
    pcl_conversions
    rosbag
    tf2_ros
    tf2_eigen
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
  DEPENDS
    EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(move_node src/move_node.cpp)
target_link_libraries(move_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lift_chair_node include/chair_manipulation_grasp_planning/grasp_planner.h include/chair_manipulation_grasp_planning/dual_grasp_planner.h src/grasp_planner.cpp src/dual_grasp_planner.cpp src/lift_chair_node.cpp)
target_link_libraries(lift_chair_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})