cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_detection_advanced)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  pcl_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  tf_conversions
  cv_bridge
  image_transport
)

find_package(PCL REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

# ================================================
# In case we want to create a custom grasp message
# ================================================
#
#set(MSG_DEPS
#  geometry_msgs
#)
#
#set(MSG_FILES
#  Grasp.msg
#)
#
#add_message_files(DIRECTORY msg FILES ${MSG_FILES})
#generate_messages(DEPENDENCIES ${MSG_DEPS})
#catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(create_grasp_database_node src/create_grasp_database_node.cpp)
target_link_libraries(create_grasp_database_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grasp_detection_node src/grasp_detection_node.cpp)
target_link_libraries(grasp_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})