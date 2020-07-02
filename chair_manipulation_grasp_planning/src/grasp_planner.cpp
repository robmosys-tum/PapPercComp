#include "chair_manipulation_grasp_planning/grasp_planner.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Pose.h>
#include "chair_manipulation_grasp_planning/utils.h"
#include <thread>

namespace chair_manipulation
{
GraspPlanner::GraspPlanner(const ros::NodeHandle& nh)
{
  arm_group_name_ = nh.param<std::string>("arm_group", "arm");
  gripper_group_name_ = nh.param<std::string>("gripper_group", "gripper");
  open_group_state_ = nh.param<std::string>("open_group_state", "open");
  closed_group_state_ = nh.param<std::string>("closed_group_state", "closed");

  world_frame_ = nh.param<std::string>("world_frame", "world");
  ik_frame_ = nh.param<std::string>("ik_frame", "wrist_3_link");
  tcp_frame_ = nh.param<std::string>("tcp_frame", "tcp");
  grasp_frame_ = nh.param<std::string>("grasp_frame", "grasp");
  grasp_tcp_aligned_frame_ = nh.param<std::string>("grasp_tcp_aligned_frame", "grasp_tcp_aligned");
  planned_pre_grasp_frame_ = nh.param<std::string>("planned_pre_grasp_frame", "planned_pre_grasp");
  planned_grasp_frame_ = nh.param<std::string>("planned_grasp_frame", "planned_grasp");
  planned_lift_frame_ = nh.param<std::string>("planned_lift_frame", "planned_lift");

  planning_attempts_ = nh.param<int>("planning_attempts", 10);
  planning_attempt_time_ = nh.param<double>("planning_attempt_time", 5.);
  pre_grasp_distance_ = nh.param<double>("pre_grasp_distance", 0.1);
  lift_height_ = nh.param<double>("lift_height", 0.2);
  max_velocity_scaling_factor_ = nh.param<double>("max_velocity_scaling_factor", 0.1);

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  arm_group_ = std::make_unique<MoveGroupInterface>(arm_group_name_);
  gripper_group_ = std::make_unique<MoveGroupInterface>(gripper_group_name_);
  planning_scene_interface_ = std::make_unique<PlanningSceneInterface>();
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

    msg = tf_buffer.lookupTransform(world_frame_, grasp_frame_, ros::Time{ 0 }, ros::Duration{ 3. });
    tf2::fromMsg(msg.transform, world_to_grasp);

    msg = tf_buffer.lookupTransform(grasp_frame_, grasp_tcp_aligned_frame_, ros::Time{ 0 }, ros::Duration{ 3. });
    tf2::fromMsg(msg.transform, grasp_to_grasp_tcp_aligned);

    msg = tf_buffer.lookupTransform(tcp_frame_, ik_frame_, ros::Time{ 0 }, ros::Duration{ 3. });
    tf2::fromMsg(msg.transform, tcp_to_ik);

    grasp_to_ik = grasp_to_grasp_tcp_aligned * tcp_to_ik;

    // The pre-grasp position is located by translating pre_grasp_distance along the z-axis of the grasp pose
    grasp_to_pre_grasp.setIdentity();
    grasp_to_pre_grasp.setOrigin(tf2::Vector3{ 0., 0., -pre_grasp_distance_ });
    world_to_pre_grasp = world_to_grasp * grasp_to_pre_grasp;

    // The lift position is just lift_height along the z-axis of the world
    world_to_lift.setRotation(world_to_grasp.getRotation());
    world_to_lift.setOrigin(world_to_grasp.getOrigin() + tf2::Vector3{ 0., 0., lift_height_ });

    // Express everything in the IK frame because Moveit's target pose is the pose of the ik_frame
    world_to_grasp_to_ik_ = world_to_grasp * grasp_to_ik;
    world_to_pre_grasp_to_ik_ = world_to_pre_grasp * grasp_to_ik;
    world_to_lift_to_ik_ = world_to_lift * grasp_to_ik;

    // Send planned transforms for visualization
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = world_frame_;

    msg.child_frame_id = planned_pre_grasp_frame_;
    tf2::convert(world_to_pre_grasp_to_ik_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = planned_grasp_frame_;
    tf2::convert(world_to_grasp_to_ik_, msg.transform);
    broadcaster_.sendTransform(msg);

    msg.child_frame_id = planned_lift_frame_;
    tf2::convert(world_to_lift_to_ik_, msg.transform);
    broadcaster_.sendTransform(msg);
  }
  catch (const tf2::TransformException& e)
  {
    throw GraspPlanningException{ "Failed to retrieve grasp pose.\nReason:\n " + std::string{ e.what() } };
  }
}

void GraspPlanner::planPreGrasp()
{
  planArmPose(world_to_pre_grasp_to_ik_, "pre-grasp");
}

void GraspPlanner::executePreGrasp()
{
  gripper_group_->setNamedTarget(open_group_state_);
  gripper_group_->move();

  auto result = arm_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute pre-grasp." };
}

void GraspPlanner::planGrasp()
{
  planArmPose(world_to_grasp_to_ik_, "grasp");
}

void GraspPlanner::executeGrasp()
{
  auto result = arm_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute grasp." };

  gripper_group_->setNamedTarget(closed_group_state_);
  gripper_group_->move();

  // Wait a couple of seconds, the grasp execution is likely to fail because it doesn't reach the closed
  // position entirely. This is absolutely fine and we just wait until the gripper exerts enough force.
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(5s);
}

void GraspPlanner::planLift()
{
  planArmPose(world_to_lift_to_ik_, "lift");
}

void GraspPlanner::executeLift()
{
  auto result = arm_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute lift." };
}

void GraspPlanner::stop()
{
  arm_group_->stop();
  gripper_group_->stop();
}

void GraspPlanner::planArmPose(const tf2::Transform& pose_tf, const std::string& pose_name)
{
  // Weird Moveit bug? - If we put addGroundPlane() in the constructor, it doesn't add
  // the ground plane to the planning scene. If we put it here, however, it works.
  // This seems to be strange...
  addGroundPlane(*planning_scene_interface_, arm_group_->getPlanningFrame());

  geometry_msgs::Pose pose;
  tf2::toMsg(pose_tf, pose);
  arm_group_->setPoseReferenceFrame(world_frame_);
  arm_group_->setPoseTarget(pose);
  arm_group_->setPlanningTime(planning_attempt_time_);
  arm_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);

  for (int i = 0; i < planning_attempts_; i++)
  {
    auto result = arm_group_->plan(plan_);
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      return;
    }
  }

  std::ostringstream msg;
  msg << "Failed to plan " << pose_name << " pose.";
  throw GraspPlanningException{ msg.str() };
}

}  // namespace chair_manipulation
