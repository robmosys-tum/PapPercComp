#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H

#include <ros/ros.h>

namespace chair_manipulation
{

class GraspPlanner
{
public:
    explicit GraspPlanner(ros::NodeHandle &nh)
    {
        arm_group = nh.param<std::string>("arm_group", "arm");
        gripper_group = nh.param<std::string>("gripper_group", "gripper");
        world_frame = nh.param<std::string>("world_frame", "world");
        grasp_frame = nh.param<std::string>("grasp_frame", "grasp");
    }

    void retrieve_grasp_pose();

    void plan_pre_grasp();

    void execute_pre_grasp();

    void plan_grasp();

    void execute_grasp();

    void plan_lift();

    void execute_lift();


private:
    std::string arm_group;
    std::string gripper_group;
    std::string world_frame;
    std::string grasp_frame;
};

class GraspPlanningException : public std::exception
{
};

}

#endif //CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
