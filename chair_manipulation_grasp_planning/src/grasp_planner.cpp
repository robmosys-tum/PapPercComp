#include "chair_manipulation_grasp_planning/grasp_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include "chair_manipulation_grasp_planning/utils.h"
#include <thread>

namespace chair_manipulation
{

GraspPlanner::GraspPlanner(const ros::NodeHandle &nh)
{
    arm_group_name = nh.param<std::string>("arm_group", "arm");
    gripper_group_name = nh.param<std::string>("gripper_group", "gripper");
    open_group_state = nh.param<std::string>("open_group_state", "open");
    closed_group_state = nh.param<std::string>("closed_group_state", "closed");

    world_frame = nh.param<std::string>("world_frame", "world");
    ik_frame = nh.param<std::string>("ik_frame", "wrist_3_link");
    tcp_frame = nh.param<std::string>("tcp_frame", "tcp");
    grasp_frame = nh.param<std::string>("grasp_frame", "grasp");
    grasp_tcp_aligned_frame = nh.param<std::string>("grasp_tcp_aligned_frame", "grasp_tcp_aligned");
    planned_pre_grasp_frame = nh.param<std::string>("planned_pre_grasp_frame", "planned_pre_grasp");
    planned_grasp_frame = nh.param<std::string>("planned_grasp_frame", "planned_grasp");
    planned_lift_frame = nh.param<std::string>("planned_lift_frame", "planned_lift");

    planning_attempts = nh.param<int>("planning_attempts", 10);
    planning_attempt_time = nh.param<double>("planning_attempt_time", 5.);
    pre_grasp_distance = nh.param<double>("pre_grasp_distance", 0.1);
    lift_height = nh.param<double>("lift_height", 0.2);
    max_velocity_scaling_factor = nh.param<double>("max_velocity_scaling_factor", 0.1);

    using moveit::planning_interface::MoveGroupInterface;
    using moveit::planning_interface::PlanningSceneInterface;
    arm_group = std::make_unique<MoveGroupInterface>(arm_group_name);
    gripper_group = std::make_unique<MoveGroupInterface>(gripper_group_name);
    planning_scene_interface = std::make_unique<PlanningSceneInterface>();
}

void GraspPlanner::prepare()
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        // Declare all transforms that we need
        tf2::Transform world_to_grasp;
        tf2::Transform world_to_pre_grasp;
        tf2::Transform world_to_lift;

        tf2::Transform grasp_to_pre_grasp;

        tf2::Transform grasp_to_grasp_tcp_aligned;
        tf2::Transform tcp_to_ik;
        tf2::Transform grasp_to_ik;

        // Retrieve transforms
        geometry_msgs::TransformStamped msg;

        msg = tf_buffer.lookupTransform(world_frame, grasp_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, world_to_grasp);

        msg = tf_buffer.lookupTransform(grasp_frame, grasp_tcp_aligned_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, grasp_to_grasp_tcp_aligned);

        msg = tf_buffer.lookupTransform(tcp_frame, ik_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, tcp_to_ik);

        grasp_to_ik = grasp_to_grasp_tcp_aligned * tcp_to_ik;

        // The pre-grasp position is located by translating pre_grasp_distance along the z-axis of the grasp pose
        grasp_to_pre_grasp.setIdentity();
        grasp_to_pre_grasp.setOrigin(tf2::Vector3{0., 0., -pre_grasp_distance});
        world_to_pre_grasp = world_to_grasp * grasp_to_pre_grasp;

        // The lift position is just lift_height along the z-axis of the world
        world_to_lift.setRotation(world_to_grasp.getRotation());
        world_to_lift.setOrigin(world_to_grasp.getOrigin() + tf2::Vector3{0., 0., lift_height});

        // Express everything in the IK frame because Moveit's target pose is the pose of the ik_frame
        world_to_grasp_to_ik = world_to_grasp * grasp_to_ik;
        world_to_pre_grasp_to_ik = world_to_pre_grasp * grasp_to_ik;
        world_to_lift_to_ik = world_to_lift * grasp_to_ik;

        // Send planned transforms for visualization
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame;

        msg.child_frame_id = planned_pre_grasp_frame;
        tf2::convert(world_to_pre_grasp_to_ik, msg.transform);
        broadcaster.sendTransform(msg);

        msg.child_frame_id = planned_grasp_frame;
        tf2::convert(world_to_grasp_to_ik, msg.transform);
        broadcaster.sendTransform(msg);

        msg.child_frame_id = planned_lift_frame;
        tf2::convert(world_to_lift_to_ik, msg.transform);
        broadcaster.sendTransform(msg);
    }
    catch (const tf2::TransformException &e)
    {
        throw GraspPlanningException{"Failed to retrieve grasp pose.\nReason:\n " + std::string{e.what()}};
    }
}

void GraspPlanner::plan_pre_grasp()
{
    plan_arm_pose(world_to_pre_grasp_to_ik, "pre-grasp");
}

void GraspPlanner::execute_pre_grasp()
{
    gripper_group->setNamedTarget(open_group_state);
    gripper_group->move();

    auto result = arm_group->execute(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute pre-grasp."};
}

void GraspPlanner::plan_grasp()
{
    plan_arm_pose(world_to_grasp_to_ik, "grasp");
}

void GraspPlanner::execute_grasp()
{
    auto result = arm_group->execute(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute grasp."};

    gripper_group->setNamedTarget(closed_group_state);
    gripper_group->move();

    // Wait a couple of seconds, the grasp execution is likely to fail because it doesn't reach the closed
    // position entirely. This is absolutely fine and we just wait until the gripper exerts enough force.
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(5s);
}

void GraspPlanner::plan_lift()
{
    plan_arm_pose(world_to_lift_to_ik, "lift");
}

void GraspPlanner::execute_lift()
{
    auto result = arm_group->execute(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute lift."};
}

void GraspPlanner::stop()
{
    arm_group->stop();
    gripper_group->stop();
}

void GraspPlanner::plan_arm_pose(const tf2::Transform &pose_tf, const std::string &pose_name)
{
    // Weird Moveit bug? - If we put add_ground_plane() in the constructor, it doesn't add
    // the ground plane to the planning scene. If we put it here, however, it works.
    // This seems to be strange...
    add_ground_plane(*planning_scene_interface, arm_group->getPlanningFrame());

    geometry_msgs::Pose pose;
    tf2::toMsg(pose_tf, pose);
    arm_group->setPoseReferenceFrame(world_frame);
    arm_group->setPoseTarget(pose);
    arm_group->setPlanningTime(planning_attempt_time);
    arm_group->setMaxVelocityScalingFactor(max_velocity_scaling_factor);

    for (int i = 0; i < planning_attempts; i++)
    {
        auto result = arm_group->plan(plan);
        if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
        {
            return;
        }
    }

    std::ostringstream msg;
    msg << "Failed to plan " << pose_name << " pose.";
    throw GraspPlanningException{msg.str()};
}

}
