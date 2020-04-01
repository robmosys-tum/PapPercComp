cmake_minimum_required(VERSION 3.5.0)
project(face_recognition)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(services REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src-gen
)

add_executable(Face_recognition src-gen/Face_recognitionCompdef/Face_recognition.cpp)
ament_target_dependencies(Face_recognition rclcpp rclcpp_lifecycle services)
ament_target_dependencies(Face_recognition sensor_msgs)
ament_target_dependencies(Face_recognition geometry_msgs)
#	target_link_libraries(Face_recognition ${catkin_LIBRARIES})

install(TARGETS
	Face_recognition
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
