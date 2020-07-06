cmake_minimum_required(VERSION 3.0.2)
project(chair_manipulation_grasp_detection_advanced)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
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

set(SOURCE_FILES
        include/chair_manipulation_grasp_detection_advanced/grasp.h
        include/chair_manipulation_grasp_detection_advanced/grasp_database.h
        include/chair_manipulation_grasp_detection_advanced/grasp_database_creator.h
        include/chair_manipulation_grasp_detection_advanced/grasp_database_element.h
        include/chair_manipulation_grasp_detection_advanced/grasp_sampler.h
        include/chair_manipulation_grasp_detection_advanced/model.h
        include/chair_manipulation_grasp_detection_advanced/utils.h
        include/chair_manipulation_grasp_detection_advanced/exception.h
        include/chair_manipulation_grasp_detection_advanced/wrench_space.h
        src/model.cpp
        src/utils.cpp
        src/grasp_database.cpp
        src/grasp_database_creator.cpp
        src/grasp_sampler.cpp
)

add_executable(create_grasp_database_node src/create_grasp_database_node.cpp ${SOURCE_FILES})
target_link_libraries(create_grasp_database_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grasp_detection_node src/grasp_detection_node.cpp ${SOURCE_FILES})
target_link_libraries(grasp_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(convert_mesh_test test/convert_mesh_test.cpp ${SOURCE_FILES})
target_link_libraries(convert_mesh_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(grasp_sampler_test test/grasp_sampler_test.cpp ${SOURCE_FILES})
target_link_libraries(grasp_sampler_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})