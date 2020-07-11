#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H

#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_msgs/GraspExecutionAction.h>

namespace chair_manipulation
{
class DualGraspPlanner
{
public:
  using Action = chair_manipulation_msgs::GraspExecutionAction;
  using Goal = chair_manipulation_msgs::GraspExecutionGoal;
  using Result = chair_manipulation_msgs::GraspExecutionResult;

  DualGraspPlanner(const std::string& planner1_action_name, const std::string& planner2_action_name)
    : client1_(planner1_action_name, true), client2_(planner2_action_name, true)
  {
  }

  bool liftChair();

private:
  actionlib::SimpleActionClient<Action> client1_;
  actionlib::SimpleActionClient<Action> client2_;

  bool executeGoal(int goal_id, const std::string& name);
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
