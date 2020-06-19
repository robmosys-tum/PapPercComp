#include <ros/ros.h>
#include <chair_manipulation_grasp_planning/dual_grasp_planner.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dual_grasp_planner_node");

    chair_manipulation::DualGraspPlanner dual_grasp_planner;
    dual_grasp_planner.lift_chair();

    return 0;
}