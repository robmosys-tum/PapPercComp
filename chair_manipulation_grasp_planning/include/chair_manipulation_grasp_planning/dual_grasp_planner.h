#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H

#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_grasp_planning/GraspExecutionAction.h>

namespace chair_manipulation
{

class DualGraspPlanner
{
public:
    using Action = chair_manipulation_grasp_planning::GraspExecutionAction;
    using Goal = chair_manipulation_grasp_planning::GraspExecutionGoal;
    using Result = chair_manipulation_grasp_planning::GraspExecutionResult;

    DualGraspPlanner(const std::string &planner1_action_name,
                     const std::string &planner2_action_name)
            : client1(planner1_action_name, true),
              client2(planner2_action_name, true)
    {}

    bool lift_chair();

private:
    actionlib::SimpleActionClient<Action> client1;
    actionlib::SimpleActionClient<Action> client2;

    bool execute_goal(int goal_id, const std::string &name);
};

}

#endif //CHAIR_MANIPULATION_GRASP_PLANNING_DUAL_GRASP_PLANNER_H
