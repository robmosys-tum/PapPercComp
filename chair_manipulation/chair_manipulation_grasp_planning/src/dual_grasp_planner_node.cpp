#include <ros/ros.h>
#include <chair_manipulation_grasp_planning/dual_grasp_planner.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "dual_grasp_planner_node");

  ros::NodeHandle nh_priv{ "~" };
  std::string planner1_action_name = nh_priv.param<std::string>("planner1_action_ns", "planner1");
  std::string planner2_action_name = nh_priv.param<std::string>("planner2_action_ns", "planner2");

  chair_manipulation::DualGraspPlanner dual_grasp_planner{ planner1_action_name, planner2_action_name };
  dual_grasp_planner.liftChair();

  return 0;
}