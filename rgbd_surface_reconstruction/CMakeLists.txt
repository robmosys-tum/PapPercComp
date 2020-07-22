cmake_minimum_required(VERSION 3.5.0)
project(rgbd_surface_reconstruction)

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
find_package(shape_msgs REQUIRED)


# ------------------------------------------------
#                 Dependencies
# ------------------------------------------------
## CUDA
find_package(CUDA 8.0 REQUIRED)
IF (CUDA_FOUND)
    include_directories("${CUDA_INCLUDE_DIRS}")
    # Optional: Specify the arch of your CUDA hardware here
    #SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++14 -gencode arch=compute_20,code=sm_21)
ENDIF ()

## OpenCV
# Optional: Set OpenCV_DIR if you want to use a custom version of OpenCV
SET("OpenCV_DIR" "/opt/opencv/share/OpenCV")
find_package(OpenCV 3.0 REQUIRED)
if (OpenCV_INCLUDE_DIRS)
    include_directories("${OpenCV_INCLUDE_DIRS}")
    link_directories(${OpenCV_DIR}/lib)
endif (OpenCV_INCLUDE_DIRS)

## Eigen3
find_package(Eigen3 REQUIRED)
if (EIGEN3_INCLUDE_DIR)
    include_directories("${EIGEN3_INCLUDE_DIR}")
endif (EIGEN3_INCLUDE_DIR)

# ------------------------------------------------

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/src-gen
	${PROJECT_SOURCE_DIR}/lib/KinectFusionLib/include
)

add_subdirectory(lib/KinectFusionLib)

add_executable(Rgbd_surface_reconstruction src-gen/Rgbd_surface_reconstructionCompdef/Rgbd_surface_reconstruction.cpp src/Rgbd_surface_reconstructionCompdef/Rgbd_surface_reconstruction_impl.cpp src/Rgbd_surface_reconstructionCompdef/depth_camera.cpp)
target_link_libraries(Rgbd_surface_reconstruction ${OpenCV_LIBS} KinectFusion)
ament_target_dependencies(Rgbd_surface_reconstruction rclcpp rclcpp_lifecycle sensor_msgs shape_msgs)


# Start of user code dependencies
# End of user code

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})


install(DIRECTORY datasets
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
	Rgbd_surface_reconstruction
	DESTINATION lib/${PROJECT_NAME}
)
ament_package()