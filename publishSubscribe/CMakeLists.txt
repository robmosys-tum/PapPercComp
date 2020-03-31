cmake_minimum_required(VERSION 3.5.0)
project(publishsubscribe)

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
find_package(simple REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
	# assure that generated .h files are found
	${PROJECT_SOURCE_DIR}/src-gen
)

add_executable(PeriodicPublisher src-gen/periodicPublisherCompdef/PeriodicPublisher.cpp)
ament_target_dependencies(PeriodicPublisher rclcpp rclcpp_lifecycle simple)
ament_target_dependencies(PeriodicPublisher "std_msgs")
#	target_link_libraries(PeriodicPublisher ${catkin_LIBRARIES})

add_executable(Subscriber src-gen/subscriberCompdef/Subscriber.cpp)
ament_target_dependencies(Subscriber rclcpp rclcpp_lifecycle simple)
ament_target_dependencies(Subscriber "std_msgs")
#	target_link_libraries(Subscriber ${catkin_LIBRARIES})

install(TARGETS
	PeriodicPublisher
	DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
	Subscriber
	DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)
ament_package()
