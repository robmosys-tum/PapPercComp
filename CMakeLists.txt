cmake_minimum_required(VERSION 3.5.0)
project(fruit_detection)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(
	${PROJECT_NAME}
	"msg/ClassBox.msg"
	"msg/BoxesImage.msg"
	"srv/Classification.srv"
	"srv/Detection.srv"
  	DEPENDENCIES sensor_msgs
	DEPENDENCIES builtin_interfaces
)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/src-gen
)

ament_python_install_package(scripts)

add_executable(FruitDetection src-gen/Fruit_detectionCompdef/FruitDetection.cpp src/Fruit_detectionCompdef/FruitDetection_impl.cpp)
ament_target_dependencies(FruitDetection rclcpp rclcpp_lifecycle geometry_msgs std_msgs sensor_msgs OpenCV fruit_detection cv_bridge)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  networks
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
	FruitDetection
	DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/detection_service.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/disease_service.py DESTINATION lib/${PROJECT_NAME})



ament_package()
