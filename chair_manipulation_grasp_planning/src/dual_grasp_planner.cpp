#include "chair_manipulation_grasp_planning/dual_grasp_planner.h"

namespace chair_manipulation
{

bool DualGraspPlanner::lift_chair()
{
    client1.waitForServer();
    client2.waitForServer();

    return execute_goal(Goal::PREPARE, "PREPARE") &&
           execute_goal(Goal::PLAN_PRE_GRASP, "PLAN_PRE_GRASP") &&
           execute_goal(Goal::EXECUTE_PRE_GRASP, "EXECUTE_PRE_GRASP") &&
           execute_goal(Goal::PLAN_GRASP, "PLAN_GRASP") &&
           execute_goal(Goal::EXECUTE_GRASP, "EXECUTE_GRASP") &&
           execute_goal(Goal::PLAN_LIFT, "PLAN_LIFT") &&
           execute_goal(Goal::EXECUTE_LIFT, "EXECUTE_LIFT");
}

bool DualGraspPlanner::execute_goal(int goal_id, const std::string &name)
{
    ROS_INFO_STREAM_NAMED("dual_grasp_planner", "Start goal " << name);
    Goal goal;
    goal.goal = goal_id;
    client1.sendGoal(goal);
    client2.sendGoal(goal);
    client1.waitForResult();
    client2.waitForResult();
    auto state1 = client1.getState();
    auto state2 = client2.getState();
    if (state1 == actionlib::SimpleClientGoalState::SUCCEEDED &&
        state2 == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO_STREAM_NAMED("dual_grasp_planner", "Finished goal " << name);
        return true;
    }
    ROS_ERROR_STREAM_NAMED("dual_grasp_planner", "Failed goal " << name);
    return false;
}

}