cmake_minimum_required(VERSION 3.5.0)
project(surfacereconstruction)

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
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src
	${PROJECT_SOURCE_DIR}/src-gen
)

add_executable(SurfaceReconstruction src-gen/SurfaceReconstructionCompdef/SurfaceReconstruction.cpp src/SurfaceReconstructionCompdef/SurfaceReconstruction_impl.cpp)
ament_target_dependencies(SurfaceReconstruction rclcpp rclcpp_lifecycle shape_msgs sensor_msgs)


# Start of user code dependencies
# End of user code

install(TARGETS
	SurfaceReconstruction
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()
