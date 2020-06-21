#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <chair_manipulation_grasp_planning/GraspExecutionAction.h>
#include "chair_manipulation_grasp_planning/grasp_planner.h"

namespace chair_manipulation
{

class GraspPlannerActionServer
{
public:
    using Action = chair_manipulation_grasp_planning::GraspExecutionAction;
    using Goal = chair_manipulation_grasp_planning::GraspExecutionGoal;
    using Result = chair_manipulation_grasp_planning::GraspExecutionResult;

    explicit GraspPlannerActionServer(const std::string &name)
            : nh_priv("~"),
              planner(nh_priv),
              name(name),
              action_server(nh, name, boost::bind(&GraspPlannerActionServer::callback, this, _1), false)
    {
        action_server.start();
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
    std::string name;
    GraspPlanner planner;
    actionlib::SimpleActionServer<Action> action_server;

    void callback(const Goal::ConstPtr &goal)
    {
        bool success = false;
        try
        {
            switch (goal->goal)
            {
                case Goal::PREPARE:
                    planner.prepare();
                    success = true;
                    break;
                case Goal::PLAN_PRE_GRASP:
                    planner.plan_pre_grasp();
                    success = true;
                    break;
                case Goal::EXECUTE_PRE_GRASP:
                    planner.execute_pre_grasp();
                    success = true;
                    break;
                case Goal::PLAN_GRASP:
                    planner.plan_grasp();
                    success = true;
                    break;
                case Goal::EXECUTE_GRASP:
                    planner.execute_grasp();
                    success = true;
                    break;
                case Goal::PLAN_LIFT:
                    planner.plan_lift();
                    success = true;
                    break;
                case Goal::EXECUTE_LIFT:
                    planner.execute_lift();
                    success = true;
                    break;
                default:
                    ROS_ERROR_NAMED(name, "Unknown goal!");
                    break;
            }
        }
        catch (const GraspPlanningException &e)
        {
            ROS_ERROR_STREAM_NAMED(name, e.what());
        }

        if (success)
        {
            action_server.setSucceeded(Result{});
        }
        else
        {
            action_server.setAborted(Result{});
        }
    }
};

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "grasp_planner_action_server_node");

    ros::NodeHandle nh_priv{"~"};
    auto action_name = nh_priv.param<std::string>("action_name", "grasp_planner");
    
    chair_manipulation::GraspPlannerActionServer server{action_name};
    ros::spin();

    return 0;
}
