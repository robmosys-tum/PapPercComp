#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

namespace chair_manipulation
{

class GraspPlanner
{
public:
    explicit GraspPlanner(const ros::NodeHandle &nh);

    void prepare();

    void plan_pre_grasp();

    void execute_pre_grasp();

    void plan_grasp();

    void execute_grasp();

    void plan_lift();

    void execute_lift();

    void stop();

private:
    std::string arm_group_name;
    std::string gripper_group_name;

    std::string base_frame;
    std::string ik_frame;
    std::string tcp_frame;
    std::string grasp_frame;
    std::string grasp_tcp_aligned_frame;

    std::string open_group_state;
    std::string closed_group_state;
    double pre_grasp_distance;
    double lift_height;

    // std::unique_ptr because we need to initialize them with values from the parameter server
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    tf2::Transform base_to_pre_grasp_to_ik;
    tf2::Transform base_to_grasp_to_ik;
    tf2::Transform base_to_lift_to_ik;

    tf2_ros::StaticTransformBroadcaster broadcaster;

    void plan_arm_pose(const tf2::Transform &pose_tf, const std::string &pose_name);
};

class GraspPlanningException : public std::runtime_error
{
public:
    explicit GraspPlanningException(const std::string &msg) : std::runtime_error(msg)
    {}
};

}

#endif //CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
