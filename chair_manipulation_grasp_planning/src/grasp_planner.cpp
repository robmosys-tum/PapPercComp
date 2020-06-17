#include "chair_manipulation_grasp_planning/grasp_planner.h"

namespace chair_manipulation
{

GraspPlanner::GraspPlanner(ros::NodeHandle &nh)
{
    arm_group_name = nh.param<std::string>("arm_group", "arm");
    gripper_group_name = nh.param<std::string>("gripper_group", "gripper");
    world_frame = nh.param<std::string>("world_frame", "world");
    grasp_frame = nh.param<std::string>("grasp_frame", "grasp");
    open_group_state = nh.param<std::string>("open_group_state", "open");
    closed_group_state = nh.param<std::string>("closed_group_state", "closed");
    auto ns = nh.param<std::string>("ns", "");
    auto planning_time = nh.param<double>("planning_time", 10.);

    group_nh = std::make_unique<ros::NodeHandle>(ns);

    using moveit::planning_interface::MoveGroupInterface;
    using moveit::planning_interface::PlanningSceneInterface;
    MoveGroupInterface::Options arm_group_options{
            arm_group_name, MoveGroupInterface::ROBOT_DESCRIPTION, *group_nh};
    MoveGroupInterface::Options gripper_group_options{
            gripper_group_name, MoveGroupInterface::ROBOT_DESCRIPTION, *group_nh};
    arm_group = std::make_unique<MoveGroupInterface>(arm_group_options);
    gripper_group = std::make_unique<MoveGroupInterface>(gripper_group_options);
    planning_scene_interface = std::make_unique<PlanningSceneInterface>(ns);

    arm_group->setPlanningTime(planning_time);
    gripper_group->setPlanningTime(planning_time);
}

void GraspPlanner::retrieve_grasp_pose()
{

}

void GraspPlanner::plan_pre_grasp()
{

}

void GraspPlanner::execute_pre_grasp()
{

}

void GraspPlanner::plan_grasp()
{

}

void GraspPlanner::execute_grasp()
{

}

void GraspPlanner::plan_lift()
{

}

void GraspPlanner::execute_lift()
{

}

}
