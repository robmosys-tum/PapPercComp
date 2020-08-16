cmake_minimum_required(VERSION 3.5.0)
project(domain_models)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src-gen
)

find_package(rosidl_default_generators REQUIRED)

## Generate messages in the 'msg' folder
rosidl_generate_interfaces(
	domain_models
	"msg/Roi.msg"
	"srv/GetInformation.srv"
	DEPENDENCIES builtin_interfaces
)

# get_default_rmw_implementation(rmw_implementation)
# find_package("${rmw_implementation}" REQUIRED)
# get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

ament_package()
