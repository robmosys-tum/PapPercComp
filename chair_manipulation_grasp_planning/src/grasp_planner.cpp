#include "chair_manipulation_grasp_planning/grasp_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include "chair_manipulation_grasp_planning/utils.h"

namespace chair_manipulation
{

GraspPlanner::GraspPlanner(ros::NodeHandle &nh)
{
    tf_prefix = nh.param<std::string>("tf_prefix", "");
    arm_group_name = nh.param<std::string>("arm_group", "arm");
    gripper_group_name = nh.param<std::string>("gripper_group", "gripper");
    world_frame = nh.param<std::string>("world_frame", "world");
    grasp_frame = nh.param<std::string>("grasp_frame", "grasp");
    open_group_state = nh.param<std::string>("open_group_state", "open");
    closed_group_state = nh.param<std::string>("closed_group_state", "closed");
    pre_grasp_distance = nh.param<double>("pre_grasp_distance", 0.1);
    lift_height = nh.param<double>("lift_height", 0.2);
    auto ns = nh.param<std::string>("ns", "");
    auto planning_time = nh.param<double>("planning_time", 10.);

    group_nh = std::make_unique<ros::NodeHandle>(ns);

    using moveit::planning_interface::MoveGroupInterface;
    using moveit::planning_interface::PlanningSceneInterface;
    MoveGroupInterface::Options arm_group_options{
            arm_group_name, MoveGroupInterface::ROBOT_DESCRIPTION, *group_nh};
    MoveGroupInterface::Options gripper_group_options{
            gripper_group_name, MoveGroupInterface::ROBOT_DESCRIPTION, *group_nh};
    arm_group = std::make_unique<MoveGroupInterface>(arm_group_options);
    gripper_group = std::make_unique<MoveGroupInterface>(gripper_group_options);
    planning_scene_interface = std::make_unique<PlanningSceneInterface>(ns);

    arm_group->setPlanningTime(planning_time);
    gripper_group->setPlanningTime(planning_time);
    arm_group->setPoseReferenceFrame(build_frame_id(world_frame));
    gripper_group->setPoseReferenceFrame(build_frame_id(world_frame));
    add_ground_plane(*arm_group, *planning_scene_interface);
    add_ground_plane(*gripper_group, *planning_scene_interface);
}

void GraspPlanner::retrieve_grasp_pose()
{
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    try
    {
        auto transform_stamped = tf_buffer.lookupTransform(build_frame_id(grasp_frame),
                                                           build_frame_id(world_frame),
                                                           ros::Time(0));
        tf2::fromMsg(transform_stamped.transform, world_to_grasp);

        // The pre-grasp position is located by translating pre_grasp_distance along the z-axis of the grasp pose
        tf2::Transform grasp_to_pre_grasp;
        grasp_to_pre_grasp.setIdentity();
        grasp_to_pre_grasp.setOrigin(tf2::Vector3{0., 0., pre_grasp_distance});
        world_to_pre_grasp = world_to_grasp * grasp_to_pre_grasp;

        // The lift position is just lift_height along the z-axis of the world
        world_to_lift.setRotation(world_to_grasp.getRotation());
        world_to_lift.setOrigin(world_to_grasp.getOrigin() + tf2::Vector3{0., 0., lift_height});

        // Publish pre-grasp and lift transforms
        tf2_ros::TransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = build_frame_id(world_frame);

        msg.child_frame_id = build_frame_id("pre_grasp");
        tf2::convert(world_to_pre_grasp, msg.transform);
        broadcaster.sendTransform(msg);

        msg.child_frame_id = build_frame_id("lift");
        tf2::convert(world_to_lift, msg.transform);
        broadcaster.sendTransform(msg);
    }
    catch (const tf2::TransformException &e)
    {
        throw GraspPlanningException{"Failed to retrieve grasp pose.\nReason:\n " + std::string{e.what()}};
    }
}

void GraspPlanner::plan_pre_grasp()
{
    plan_arm_pose(world_to_pre_grasp, "pre-grasp");
}

void GraspPlanner::execute_pre_grasp()
{
    arm_group->execute(plan);
    gripper_group->setNamedTarget(open_group_state);
    gripper_group->move();
}

void GraspPlanner::plan_grasp()
{
    plan_arm_pose(world_to_grasp, "grasp");
}

void GraspPlanner::execute_grasp()
{
    arm_group->execute(plan);
    gripper_group->setNamedTarget(closed_group_state);
    gripper_group->move();
}

void GraspPlanner::plan_lift()
{
    plan_arm_pose(world_to_lift, "lift");
}

void GraspPlanner::execute_lift()
{
    arm_group->execute(plan);
}

std::string GraspPlanner::build_frame_id(const std::string &frame) const
{
    if (tf_prefix.empty())
        return frame;

    return tf_prefix + "/" + frame;
}

void GraspPlanner::plan_arm_pose(const tf2::Transform &pose_tf, const std::string &pose_name)
{
    geometry_msgs::Pose pose;
    tf2::toMsg(pose_tf, pose);
    arm_group->setPoseTarget(pose);
    auto planning_result = arm_group->plan(plan);
    if (planning_result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
        std::ostringstream msg;
        msg << "Failed to plan " << pose_name << " pose.";
        throw GraspPlanningException{msg.str()};
    }
}

}
