#include "chair_manipulation_grasp_planning/grasp_planner.h"
#include "chair_manipulation_grasp_planning/utils.h"
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <thread>

namespace chair_manipulation
{
GraspPlanner::GraspPlanner(ros::NodeHandle& nh)
{
  arm_group_name_ = nh.param<std::string>("arm_group", "arm");
  gripper_group_name_ = nh.param<std::string>("gripper_group", "gripper");
  open_group_state_ = nh.param<std::string>("open_group_state", "open");
  closed_group_state_ = nh.param<std::string>("closed_group_state", "closed");

  world_frame_ = nh.param<std::string>("world_frame", "world");
  ik_frame_ = nh.param<std::string>("ik_frame", "wrist_3_link");
  tcp_frame_ = nh.param<std::string>("tcp_frame", "tcp");
  grasp_frame_ = nh.param<std::string>("grasp_frame", "grasp");
  planned_pre_grasp_frame_ = nh.param<std::string>("planned_pre_grasp_frame", "planned_pre_grasp");
  planned_grasp_frame_ = nh.param<std::string>("planned_grasp_frame", "planned_grasp");
  planned_lift_frame_ = nh.param<std::string>("planned_lift_frame", "planned_lift");

  object_mesh_topic_ = nh.param<std::string>("object_mesh_topic", "object_mesh");

  planning_attempts_ = nh.param<int>("planning_attempts", 10);
  planning_attempt_time_ = nh.param<double>("planning_attempt_time", 5.);
  pre_grasp_distance_ = nh.param<double>("pre_grasp_distance", 0.1);
  lift_height_ = nh.param<double>("lift_height", 0.2);
  max_velocity_scaling_factor_ = nh.param<double>("max_velocity_scaling_factor", 0.1);
  position_tolerance_ = nh.param<double>("position_tolerance", 0.01);

  touch_links_ = nh.param<std::vector<std::string>>("touch_links", {});

  gripper_command_action_ns_ = nh.param<std::string>("gripper_command_action_ns", "gripper_command");
  gripper_command_client_ = std::make_unique<GripperCommandActionClient>(gripper_command_action_ns_, true);

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;
  arm_group_ = std::make_unique<MoveGroupInterface>(arm_group_name_);
  gripper_group_ = std::make_unique<MoveGroupInterface>(gripper_group_name_);
  planning_scene_interface_ = std::make_unique<PlanningSceneInterface>();
}

void GraspPlanner::prepare()
{
  // Wait until we received the object mesh
  ROS_INFO_STREAM_NAMED("grasp_planner", "Waiting to receive object mesh...");
  object_mesh_ = ros::topic::waitForMessage<shape_msgs::Mesh>(object_mesh_topic_);
  ROS_INFO_STREAM_NAMED("grasp_planner", "Object mesh received.");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  try
  {
    ROS_INFO_STREAM_NAMED("grasp_planner", "Retrieving transform...");

    // Declare all transforms that we need
    tf2::Transform world_to_grasp;
    tf2::Transform world_to_pre_grasp;
    tf2::Transform world_to_lift;
    tf2::Transform grasp_to_pre_grasp;
    tf2::Transform tcp_to_ik;

    // Retrieve transforms
    geometry_msgs::TransformStamped msg;

    ROS_DEBUG_STREAM_NAMED("grasp_planner", "Looking up transforms.");
    msg = tf_buffer.lookupTransform(world_frame_, grasp_frame_, ros::Time{ 0 }, ros::Duration{ 120. });
    tf2::fromMsg(msg.transform, world_to_grasp);

    msg = tf_buffer.lookupTransform(tcp_frame_, ik_frame_, ros::Time{ 0 }, ros::Duration{ 120. });
    tf2::fromMsg(msg.transform, tcp_to_ik);

    // The pre-grasp position is located by translating pre_grasp_distance along
    // the z-axis of the grasp pose
    grasp_to_pre_grasp.setIdentity();
    grasp_to_pre_grasp.setOrigin(tf2::Vector3{ 0., 0., -pre_grasp_distance_ });
    world_to_pre_grasp = world_to_grasp * grasp_to_pre_grasp;

    // The lift position is just lift_height along the z-axis of the world
    world_to_lift.setRotation(world_to_grasp.getRotation());
    world_to_lift.setOrigin(world_to_grasp.getOrigin() + tf2::Vector3{ 0., 0., lift_height_ });

    // Express everything in the IK frame because Moveit's target pose is the
    // pose of the ik_frame
    world_to_grasp_to_ik_ = world_to_grasp * tcp_to_ik;
    world_to_pre_grasp_to_ik_ = world_to_pre_grasp * tcp_to_ik;
    world_to_lift_to_ik_ = world_to_lift * tcp_to_ik;

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

    ROS_INFO_STREAM_NAMED("grasp_planner", "Finished retrieving transform.");
  }
  catch (const tf2::TransformException& e)
  {
    throw GraspPlanningException{ "Failed to retrieve grasp pose.\nReason:\n " + std::string{ e.what() } };
  }
}

void GraspPlanner::planPreGrasp()
{
  arm_group_->clearPathConstraints();
  addGroundPlane(*planning_scene_interface_, world_frame_);
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
  setPathConstraints(world_to_grasp_to_ik_);
  planArmPose(world_to_grasp_to_ik_, "grasp");
}

void GraspPlanner::executeGrasp()
{
  auto result = arm_group_->execute(plan_);
  if (result != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    throw GraspPlanningException{ "Failed to execute grasp while moving arm." };

  attachMesh(*planning_scene_interface_, *arm_group_, *object_mesh_, "object_mesh", world_frame_, tcp_frame_,
             touch_links_);
  closeGripper();
}

void GraspPlanner::planLift()
{
  setPathConstraints(world_to_lift_to_ik_);
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

void GraspPlanner::cleanup()
{
  removeGroundPlane(*planning_scene_interface_);
  planning_scene_interface_->removeCollisionObjects({ "object_mesh" });
  arm_group_->clearPathConstraints();
}

void GraspPlanner::planArmPose(const tf2::Transform& pose_tf, const std::string& pose_name)
{
  geometry_msgs::Pose pose;
  tf2::toMsg(pose_tf, pose);
  arm_group_->setPoseReferenceFrame(world_frame_);
  arm_group_->setPlanningTime(planning_attempt_time_);
  arm_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
  arm_group_->setGoalTolerance(0.01);
  arm_group_->setPoseTarget(pose);

  for (int i = 0; i < planning_attempts_; i++)
  {
    auto result = arm_group_->plan(plan_);
    if (result == moveit::planning_interface::MoveItErrorCode::SUCCESS)
      return;
  }

  std::ostringstream msg;
  msg << "Failed to plan " << pose_name << " pose.";
  throw GraspPlanningException{ msg.str() };
}

void GraspPlanner::openGripper()
{
  GripperCommandGoal goal;
  goal.goal = GripperCommandGoal::OPEN;
  gripper_command_client_->sendGoal(goal);
  gripper_command_client_->waitForResult(ros::Duration{ 5. });
  if (gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to open gripper." };
}

void GraspPlanner::closeGripper()
{
  GripperCommandGoal goal;
  goal.goal = GripperCommandGoal::CLOSE;
  gripper_command_client_->sendGoal(goal);
  gripper_command_client_->waitForResult(ros::Duration{ 5. });
  if (gripper_command_client_->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    throw GraspPlanningException{ "Failed to close gripper." };
}

void GraspPlanner::setPathConstraints(const tf2::Transform& goal_pose)
{
  // Get start pose
  tf2::Transform start_pose;
  auto state = arm_group_->getCurrentState();
  Eigen::Isometry3d arm_to_world = state->getFrameTransform(world_frame_);
  Eigen::Isometry3d arm_to_ik = state->getFrameTransform(ik_frame_);
  Eigen::Isometry3d world_to_ik = arm_to_world.inverse() * arm_to_ik;
  tf2::convert(world_to_ik, start_pose);
  tf2::Vector3 start_to_goal = goal_pose.getOrigin() - start_pose.getOrigin();

  // Orientation constraint
  const auto& orientation = world_to_grasp_to_ik_.getRotation();
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = world_frame_;
  orientation_constraint.link_name = ik_frame_;
  orientation_constraint.orientation.x = orientation.x();
  orientation_constraint.orientation.y = orientation.y();
  orientation_constraint.orientation.z = orientation.z();
  orientation_constraint.orientation.w = orientation.w();
  orientation_constraint.absolute_x_axis_tolerance = 0.1;
  orientation_constraint.absolute_y_axis_tolerance = 0.1;
  orientation_constraint.absolute_z_axis_tolerance = 0.1;
  orientation_constraint.weight = 1.0;

  // Constrain position to be inside a cylinder
  shape_msgs::SolidPrimitive cylinder_primitive;
  cylinder_primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
  cylinder_primitive.dimensions.resize(2);
  cylinder_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = position_tolerance_;
  cylinder_primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] =
      start_to_goal.length() + 2. * position_tolerance_;

  // Cylinder pose
  tf2::Vector3 cylinder_position = start_pose.getOrigin() + 0.5 * start_to_goal;
  tf2::Vector3 cylinder_z_direction = start_to_goal.normalized();
  tf2::Vector3 origin_z_direction{ 0., 0., 1. };
  tf2::Vector3 axis = origin_z_direction.cross(cylinder_z_direction).normalized();
  double angle = std::acos(cylinder_z_direction.dot(origin_z_direction));
  tf2::Quaternion q{ axis, angle };
  q.normalize();
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.position.x = cylinder_position.x();
  cylinder_pose.position.y = cylinder_position.y();
  cylinder_pose.position.z = cylinder_position.z();
  cylinder_pose.orientation.x = q.x();
  cylinder_pose.orientation.y = q.y();
  cylinder_pose.orientation.z = q.z();
  cylinder_pose.orientation.w = q.w();

  // Position constraint
  moveit_msgs::PositionConstraint position_constraint;
  position_constraint.header.frame_id = world_frame_;
  position_constraint.link_name = ik_frame_;
  position_constraint.target_point_offset.x = 0.;
  position_constraint.target_point_offset.y = 0.;
  position_constraint.target_point_offset.z = 0.;
  position_constraint.weight = 1.0;
  position_constraint.constraint_region.primitives.push_back(cylinder_primitive);
  position_constraint.constraint_region.primitive_poses.push_back(cylinder_pose);

  // Add to arm group
  moveit_msgs::Constraints constraints;
  constraints.orientation_constraints.push_back(orientation_constraint);
  constraints.position_constraints.push_back(position_constraint);
  arm_group_->setPathConstraints(constraints);
}

}  // namespace chair_manipulation
