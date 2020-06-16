#include <ros/ros.h>
#include <chair_manipulation_grasp_planning/dual_grasp_planner.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lift_chair_node");

    ros::NodeHandle nh_planner1("~planner1");
    ros::NodeHandle nh_planner2("~planner2");

    chair_manipulation::GraspPlanner grasp_planner1{nh_planner1};
    chair_manipulation::GraspPlanner grasp_planner2{nh_planner2};

    chair_manipulation::DualGraspPlanner dual_grasp_planner{grasp_planner1, grasp_planner2};
    dual_grasp_planner.lift_chair();

    return 0;
}