#include "chair_manipulation_grasp_planning/dual_grasp_planner.h"

namespace chair_manipulation
{
bool DualGraspPlanner::liftChair()
{
  client1_.waitForServer();
  client2_.waitForServer();

  return executeGoal(Goal::PREPARE, "PREPARE") && executeGoal(Goal::PLAN_PRE_GRASP, "PLAN_PRE_GRASP") &&
         executeGoal(Goal::EXECUTE_PRE_GRASP, "EXECUTE_PRE_GRASP") && executeGoal(Goal::PLAN_GRASP, "PLAN_GRASP") &&
         executeGoal(Goal::EXECUTE_GRASP, "EXECUTE_GRASP") && executeGoal(Goal::PLAN_LIFT, "PLAN_LIFT") &&
         executeGoal(Goal::EXECUTE_LIFT, "EXECUTE_LIFT");
}

bool DualGraspPlanner::executeGoal(int goal_id, const std::string& name)
{
  ROS_INFO_STREAM_NAMED("dual_grasp_planner", "Start goal " << name);
  Goal goal;
  goal.goal = goal_id;
  client1_.sendGoal(goal);
  client2_.sendGoal(goal);
  client1_.waitForResult();
  client2_.waitForResult();
  auto state1 = client1_.getState();
  auto state2 = client2_.getState();
  if (state1 == actionlib::SimpleClientGoalState::SUCCEEDED && state2 == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO_STREAM_NAMED("dual_grasp_planner", "Finished goal " << name);
    return true;
  }
  ROS_ERROR_STREAM_NAMED("dual_grasp_planner", "Failed goal " << name);
  return false;
}

}  // namespace chair_manipulation