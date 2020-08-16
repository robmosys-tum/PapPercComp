cmake_minimum_required(VERSION 3.5.0)
project(garbage_classification)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Torch REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/src-gen
)

add_executable(Garbage_classification src-gen/Garbage_classificationCompdef/Garbage_classification.cpp src/Garbage_classificationCompdef/Garbage_classification_impl.cpp)
ament_target_dependencies(Garbage_classification rclcpp rclcpp_lifecycle std_msgs sensor_msgs)

target_link_libraries(Garbage_classification ${TORCH_LIBRARIES} ${OpenCV_LIBRARIES})


# Start of user code dependencies
# End of user code

install(TARGETS
	Garbage_classification
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
