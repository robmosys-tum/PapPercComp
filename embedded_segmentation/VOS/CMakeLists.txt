cmake_minimum_required(VERSION 3.5.0)
project(vos)

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
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Torch REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src
)

add_executable(VOS src/VOSCompdef/VOS.cpp)
ament_target_dependencies(VOS rclcpp rclcpp_lifecycle sensor_msgs "OpenCV" cv_bridge)


# Start of user code dependencies
# End of user code

install(TARGETS
	VOS
	DESTINATION lib/${PROJECT_NAME}
)

### Path should be to installed libtorch library
#list(APPEND CMAKE_PREFIX_PATH "~/Documents/ConversionCpp/libtorch")
target_link_libraries(VOS "${TORCH_LIBRARIES}")

ament_package()
