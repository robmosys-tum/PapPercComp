cmake_minimum_required(VERSION 3.5)
project(instance_recognition)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter -Wdeprecated-declarations -Wsign-compare)
endif()

# add_compile_options(-std=c++11)
## FIX LATER
set(rhome_metadata_FOUND 1)

SET(OpenCV_DIR /usr/local/lib/cmake/opencv4)
# </opencv_home_dir>
## - - find packages - - 
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rhome_metadata REQUIRED)
find_package(Boost 1.4.0 COMPONENTS filesystem REQUIRED)



###########
## Build ##
###########
include_directories(include ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} libs/lbplibrary )

## Declare a C++ executable
add_executable(instance_recognition src/instance_recognition.cpp )

ament_target_dependencies(
 instance_recognition
  rclcpp
  std_msgs
  sensor_msgs
  OpenCV
  image_transport
  cv_bridge
  rhome_metadata
 )
target_include_directories(instance_recognition PRIVATE include )
target_link_libraries(instance_recognition ${OpenCV_LIBS}  ${Boost_LIBRARIES} lbp )

install(TARGETS
instance_recognition
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
