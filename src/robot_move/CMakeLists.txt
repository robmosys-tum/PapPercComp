cmake_minimum_required(VERSION 3.0.2)
project(robot_move)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        moveit_visual_tools
        moveit_ros_planning_interface
        message_generation
        )

find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED system filesystem date_time thread)

add_message_files(
        FILES
        classbox.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
)


catkin_package(
        CATKIN_DEPENDS
        DEPENDS
        EIGEN3
)

include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR}  ${EIGEN3_INCLUDE_DIRS})

add_executable(robot_move src/robot-move_node.cpp)
target_link_libraries(robot_move
        ${catkin_LIBRARIES}
    )

add_executable(talker src/publisher.cpp)

target_link_libraries(talker
        ${catkin_LIBRARIES}
        )