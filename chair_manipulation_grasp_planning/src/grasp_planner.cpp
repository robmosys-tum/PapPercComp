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

    base_frame = nh.param<std::string>("base_frame", "base_link");
    ik_frame = nh.param<std::string>("ik_frame", "wrist_3_link");
    tcp_frame = nh.param<std::string>("tcp_frame", "tcp");
    grasp_frame = nh.param<std::string>("grasp_frame", "grasp");
    grasp_tcp_aligned_frame = nh.param<std::string>("grasp_tcp_aligned_frame", "grasp_tcp_aligned");

    open_group_state = nh.param<std::string>("open_group_state", "open");
    closed_group_state = nh.param<std::string>("closed_group_state", "closed");
    pre_grasp_distance = nh.param<double>("pre_grasp_distance", 0.1);
    lift_height = nh.param<double>("lift_height", 0.2);
    auto planning_time = nh.param<double>("planning_time", 10.);

    using moveit::planning_interface::MoveGroupInterface;
    using moveit::planning_interface::PlanningSceneInterface;
    arm_group = std::make_unique<MoveGroupInterface>(arm_group_name);
    gripper_group = std::make_unique<MoveGroupInterface>(gripper_group_name);
    planning_scene_interface = std::make_unique<PlanningSceneInterface>();

    arm_group->setPlanningTime(planning_time);
    gripper_group->setPlanningTime(planning_time);

    add_ground_plane(*planning_scene_interface, arm_group->getPlanningFrame());
}

void GraspPlanner::prepare()
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        // Declare all transforms that we need
        tf2::Transform base_to_grasp;
        tf2::Transform base_to_pre_grasp;
        tf2::Transform base_to_lift;

        tf2::Transform grasp_to_pre_grasp;

        tf2::Transform grasp_to_grasp_tcp_aligned;
        tf2::Transform tcp_to_ik;
        tf2::Transform grasp_to_ik;

        // Retrieve transforms
        geometry_msgs::TransformStamped msg;

        msg = tf_buffer.lookupTransform(base_frame, grasp_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, base_to_grasp);

        msg = tf_buffer.lookupTransform(grasp_frame, grasp_tcp_aligned_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, grasp_to_grasp_tcp_aligned);

        msg = tf_buffer.lookupTransform(tcp_frame, ik_frame, ros::Time{0}, ros::Duration{3.});
        tf2::fromMsg(msg.transform, tcp_to_ik);

        grasp_to_ik = grasp_to_grasp_tcp_aligned * tcp_to_ik;

        // The pre-grasp position is located by translating pre_grasp_distance along the z-axis of the grasp pose
        grasp_to_pre_grasp.setIdentity();
        grasp_to_pre_grasp.setOrigin(tf2::Vector3{0., 0., -pre_grasp_distance});
        base_to_pre_grasp = base_to_grasp * grasp_to_pre_grasp;

        // The lift position is just lift_height along the z-axis of the base
        base_to_lift.setRotation(base_to_grasp.getRotation());
        base_to_lift.setOrigin(base_to_grasp.getOrigin() + tf2::Vector3{0., 0., lift_height});

        // Express everything in the IK frame because Moveit's target pose is the pose of the ik_frame
        base_to_grasp_to_ik = base_to_grasp * grasp_to_ik;
        base_to_pre_grasp_to_ik = base_to_pre_grasp * grasp_to_ik;
        base_to_lift_to_ik = base_to_lift * grasp_to_ik;
    }
    catch (const tf2::TransformException &e)
    {
        throw GraspPlanningException{"Failed to retrieve grasp pose.\nReason:\n " + std::string{e.what()}};
    }
}

void GraspPlanner::plan_pre_grasp()
{
    plan_arm_pose(base_to_pre_grasp_to_ik, "pre-grasp");
}

void GraspPlanner::execute_pre_grasp()
{
    auto result = arm_group->execute(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute pre-grasp."};

    gripper_group->setNamedTarget(open_group_state);
    result = gripper_group->move();
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute pre-grasp."};
}

void GraspPlanner::plan_grasp()
{
    plan_arm_pose(base_to_grasp_to_ik, "grasp");
}

void GraspPlanner::execute_grasp()
{
    auto result = arm_group->execute(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute grasp."};

    gripper_group->setNamedTarget(closed_group_state);
    result = gripper_group->move();
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
        throw GraspPlanningException{"Failed to execute grasp."};
}

void GraspPlanner::plan_lift()
{
    plan_arm_pose(base_to_lift_to_ik, "lift");
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
    geometry_msgs::Pose pose;
    tf2::toMsg(pose_tf, pose);
    arm_group->setPoseTarget(pose);
    auto result = arm_group->plan(plan);
    if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        std::ostringstream msg;
        msg << "Failed to plan " << pose_name << " pose.";
        throw GraspPlanningException{msg.str()};
    }
}

}
