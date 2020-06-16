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
        future1 = std::async(std::launch::async, [&]
        { planner1.retrieve_grasp_pose(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.retrieve_grasp_pose(); });
        future1.get();
        future2.get();

        // Plan pre-grasp
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_pre_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_pre_grasp(); });
        future1.get();
        future2.get();

        // Execute pre-grasp
        future1 = std::async(std::launch::async, [&]
        { planner1.execute_pre_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.execute_pre_grasp(); });
        future1.get();
        future2.get();

        // Plan grasp
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_grasp(); });
        future1.get();
        future2.get();

        // Execute grasp
        future1 = std::async(std::launch::async, [&]
        { planner1.execute_grasp(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.execute_grasp(); });
        future1.get();
        future2.get();

        // Plan lift
        future1 = std::async(std::launch::async, [&]
        { planner1.plan_lift(); });
        future2 = std::async(std::launch::async, [&]
        { planner2.plan_lift(); });
        future1.get();
        future2.get();

        // Plan grasp
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