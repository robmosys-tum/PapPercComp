#ifndef CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
#define CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/Mesh.h>
#include <std_msgs/String.h>
#include <mutex>
#include <actionlib/client/simple_action_client.h>
#include <chair_manipulation_msgs/GripperCommandAction.h>

namespace chair_manipulation
{
class GraspPlanner
{
public:
  using GripperCommandAction = chair_manipulation_msgs::GripperCommandAction;
  using GripperCommandGoal = chair_manipulation_msgs::GripperCommandGoal;
  using GripperCommandResult = chair_manipulation_msgs::GripperCommandResult;
  using GripperCommandActionClient = actionlib::SimpleActionClient<GripperCommandAction>;

  explicit GraspPlanner(ros::NodeHandle& nh);

  void prepare();

  void planPreGrasp();

  void executePreGrasp();

  void planGrasp();

  void executeGrasp();

  void planLift();

  void executeLift();

  void stop();

  void cleanup();

private:
  std::string arm_group_name_;
  std::string gripper_group_name_;
  std::string open_group_state_;
  std::string closed_group_state_;

  std::string world_frame_;
  std::string ik_frame_;
  std::string tcp_frame_;
  std::string grasp_frame_;
  std::string planned_pre_grasp_frame_;
  std::string planned_grasp_frame_;
  std::string planned_lift_frame_;

  std::string object_mesh_topic_;

  int planning_attempts_;
  double planning_attempt_time_;
  double pre_grasp_distance_;
  double lift_height_;
  double max_velocity_scaling_factor_;

  std::vector<std::string> touch_links_;

  std::string gripper_command_action_ns_;
  std::unique_ptr<GripperCommandActionClient> gripper_command_client_;

  // std::unique_ptr because we need to initialize them with values from the parameter server
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

  moveit::planning_interface::MoveGroupInterface::Plan plan_;

  tf2::Transform world_to_pre_grasp_to_ik_;
  tf2::Transform world_to_grasp_to_ik_;
  tf2::Transform world_to_lift_to_ik_;

  tf2_ros::StaticTransformBroadcaster broadcaster_;

  shape_msgs::MeshConstPtr object_mesh_;

  void planArmPose(const tf2::Transform& pose_tf, const std::string& pose_name);

  void openGripper();

  void closeGripper();

  void constrainOrientation();
};

class GraspPlanningException : public std::runtime_error
{
public:
  explicit GraspPlanningException(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_PLANNING_GRASP_PLANNER_H
