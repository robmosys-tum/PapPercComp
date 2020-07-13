cmake_minimum_required(VERSION 3.5)
project(object_pose_estimation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter -Wdeprecated-declarations -Wsign-compare)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL 1.11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenNI REQUIRED)

# link_directories( ${PCL_LIBRARY_DIRS} )
# add_definitions( ${PCL_DEFINITIONS} )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES object_pose_estimation
# )


include_directories( 
	include
	${EIGEN_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OPENNI_INCLUDE_DIRS}
#   /usr/include/pcl-1.7
#   /usr/include/ni
)

# ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
    rclcpp
)

# Setting stuff
set( fitter_src
  src/InertiaCalculations.cpp
  src/Minimization.cpp
  src/OPESettings.cpp
  src/OPEMain.cpp
  src/ObjectPoseEstimator.cpp
  src/Plane.cpp
  src/PointCloudCapture.cpp
  src/OPEUtils.cpp
  src/SQFitting.cpp
)

# set( fitter_hdr
#   include/object_pose_estimation/OPECommon.h 
#   include/object_pose_estimation/OPESettings.h 
#   include/object_pose_estimation/EigenSolver.h 
#   include/object_pose_estimation/InertiaCalculations.h 
#   include/object_pose_estimation/Minimization.h 
#   include/object_pose_estimation/ObjectPoseEstimator.h
#   include/object_pose_estimation/Plane.h 
#   include/object_pose_estimation/PointCloudCapture.h 
#   include/object_pose_estimation/SQFitting.h 
#   include/object_pose_estimation/SQTypes.h 
#   include/object_pose_estimation/TableObjectDetector.h
#   include/object_pose_estimation/TableObjectDetector.hpp
#   include/object_pose_estimation/TableObjectModeler.h 
#   include/object_pose_estimation/OPEUtils.h 
# )

# Add library
add_library( ${PROJECT_NAME} ${fitter_src})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
 )

target_link_libraries( ${PROJECT_NAME} ${OPENNI_LIBRARIES} ${PCL_LIBRARIES} ) 


install(DIRECTORY include/
  DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(
  rclcpp)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})


ament_package()


