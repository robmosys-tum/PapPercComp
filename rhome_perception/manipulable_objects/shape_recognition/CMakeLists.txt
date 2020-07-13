cmake_minimum_required(VERSION 3.5)
project(shape_recognition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter -Wdeprecated-declarations -Wsign-compare)
endif()


## FIX LATER
set(rhome_metadata_FOUND 1)

SET(OpenCV_DIR /usr/local/lib/cmake/opencv4)
# </opencv_home_dir>
## - - find packages - - 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rhome_metadata REQUIRED)
find_package(Boost 1.4.0 COMPONENTS filesystem REQUIRED)

find_package(object_pose_estimation REQUIRED)
find_package(PCL 1.7 REQUIRED)


###########
## Build ##
###########
include_directories(include ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} src/shape_recognition.cpp )

ament_target_dependencies(
 ${PROJECT_NAME}
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  OpenCV
  image_transport
  cv_bridge
  rhome_metadata
  PCL
  object_pose_estimation
 )
target_include_directories(${PROJECT_NAME} PRIVATE include )
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${Boost_LIBRARIES} )

install(TARGETS
${PROJECT_NAME}
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
