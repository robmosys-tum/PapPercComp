#include "chair_manipulation_grasp_planning/dual_grasp_planner.h"
#include <future>

namespace chair_manipulation
{

void chair_manipulation::DualGraspPlanner::lift_chair()
{
    try
    {
        std::future<void> future1, future2;

        // Retrieve grasp pose
        ROS_INFO_STREAM_NAMED("grasp_planning", "Retrieve grasp poses.");
        future1 = std::async(std::launch::async, [&]
        { planner1.retrieve_grasp_pose(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.retrieve_grasp_pose(); });
        future1.get();
        future2.get();

        // Plan pre-grasp
        ROS_INFO_STREAM_NAMED("grasp_planning", "Plan pre-grasp.");
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_pre_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_pre_grasp(); });
        future1.get();
        future2.get();

        // Execute pre-grasp
        ROS_INFO_STREAM_NAMED("grasp_planning", "Execute pre-grasp.");
        future1 = std::async(std::launch::async, [&]
        { planner1.execute_pre_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.execute_pre_grasp(); });
        future1.get();
        future2.get();

        // Plan grasp
        ROS_INFO_STREAM_NAMED("grasp_planning", "Plan grasp.");
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_grasp(); });
        future1.get();
        future2.get();

        // Execute grasp
        ROS_INFO_STREAM_NAMED("grasp_planning", "Execute grasp.");
        future1 = std::async(std::launch::async, [&]
        { planner1.execute_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.execute_grasp(); });
        future1.get();
        future2.get();

        // Plan lift
        ROS_INFO_STREAM_NAMED("grasp_planning", "Plan lift.");
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_lift(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_lift(); });
        future1.get();
        future2.get();

        // Plan grasp
        ROS_INFO_STREAM_NAMED("grasp_planning", "Execute lift.");
        future1 = std::async(std::launch::async, [&]
        { planner1.execute_lift(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.execute_lift(); });
        future1.get();
        future2.get();
    }
    catch (const GraspPlanningException &e)
    {
        ROS_ERROR_STREAM_NAMED("grasp_planning", e.what());
    }
}

}