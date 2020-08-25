#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <chair_manipulation_msgs/GraspExecutionAction.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

namespace chair_manipulation
{
class GraspPlannerActionServer
{
public:
  using Action = chair_manipulation_msgs::GraspExecutionAction;
  using Goal = chair_manipulation_msgs::GraspExecutionGoal;
  using Result = chair_manipulation_msgs::GraspExecutionResult;

  explicit GraspPlannerActionServer(const std::string& action_ns)
    : nh_priv_("~")
    , planner_(nh_priv_)
    , action_server_(nh_, action_ns, boost::bind(&GraspPlannerActionServer::callback, this, _1), false)
  {
    action_server_.start();
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  GraspPlanner planner_;
  actionlib::SimpleActionServer<Action> action_server_;

  void callback(const Goal::ConstPtr& goal)
  {
    bool success = false;
    try
    {
      switch (goal->goal)
      {
        case Goal::PREPARE:
          planner_.prepare();
          success = true;
          break;
        case Goal::PLAN_PRE_GRASP:
          planner_.planPreGrasp();
          success = true;
          break;
        case Goal::EXECUTE_PRE_GRASP:
          planner_.executePreGrasp();
          success = true;
          break;
        case Goal::PLAN_GRASP:
          planner_.planGrasp();
          success = true;
          break;
        case Goal::EXECUTE_GRASP:
          planner_.executeGrasp();
          success = true;
          break;
        case Goal::PLAN_LIFT:
          planner_.planLift();
          success = true;
          break;
        case Goal::EXECUTE_LIFT:
          planner_.executeLift();
          success = true;
          break;
        default:
          ROS_ERROR_STREAM_NAMED("grasp_planner_action_server", "Unknown goal.");
          break;
      }
    }
    catch (const GraspPlanningException& e)
    {
      ROS_ERROR_STREAM_NAMED("grasp_planner_action_server", e.what());
    }

    if (!success || goal->goal == Goal::EXECUTE_LIFT)
      planner_.cleanup();

    if (success)
      action_server_.setSucceeded(Result{});
    else
      action_server_.setAborted(Result{});
  }
};

}  // namespace chair_manipulation

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_planner_action_server_node");
  ros::AsyncSpinner spinner{1};
  spinner.start();

  ros::NodeHandle nh_priv{ "~" };
  auto action_name = nh_priv.param<std::string>("action_ns", "grasp_planner");

  chair_manipulation::GraspPlannerActionServer server{ action_name };

  ros::waitForShutdown();
  return 0;
}
