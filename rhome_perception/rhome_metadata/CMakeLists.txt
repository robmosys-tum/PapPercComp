cmake_minimum_required(VERSION 3.5)
project(rhome_metadata)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
    "msg/Face.msg"
    "msg/Rect.msg"
    "msg/Roi.msg"
    "msg/Color.msg"
    "msg/Shape.msg"
)


set(srv_files
    "srv/Getinformation.srv"      
    "srv/Objectinformation.srv"        
    "srv/Onoff.srv"
    "srv/Position3d.srv"               
    "srv/Queryobject.srv"
    "srv/Recognize.srv"     
)


# Actually generate the language-specific message and service files
rosidl_generate_interfaces(rhome_metadata
    ${msg_files}
    ${srv_files}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

#ament_export_dependencies(rosidl_default_runtime)
#ament_export_include_directories(include)

ament_export_include_directories(include)
ament_export_dependencies(${PROJECT_NAME})
ament_export_dependencies(
    rosidl_default_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_package()
