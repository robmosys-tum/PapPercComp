#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>

namespace chair_manipulation
{

class GraspPlanner
{
public:
    explicit GraspPlanner(ros::NodeHandle &nh);

    void retrieve_grasp_pose();

    void plan_pre_grasp();

    void execute_pre_grasp();

    void plan_grasp();

    void execute_grasp();

    void plan_lift();

    void execute_lift();

private:
    std::string tf_prefix;
    std::string arm_group_name;
    std::string gripper_group_name;
    std::string world_frame;
    std::string grasp_frame;
    std::string open_group_state;
    std::string closed_group_state;
    double pre_grasp_distance;
    double lift_height;

    // std::unique_ptr because we need to initialize them with values from the parameter server
    std::unique_ptr<ros::NodeHandle> group_nh;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    tf2::Transform world_to_pre_grasp;
    tf2::Transform world_to_grasp;
    tf2::Transform world_to_lift;

    std::string build_frame_id(const std::string &frame) const;

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
