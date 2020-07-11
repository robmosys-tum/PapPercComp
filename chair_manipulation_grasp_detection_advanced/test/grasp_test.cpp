#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/model.h"
#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <string>
#include <fstream>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace chair_manipulation;

std::string loadFileContent(const std::string& filename)
{
  std::ifstream t(filename);
  return std::string((std::istreambuf_iterator<char>(t)), std::istreambuf_iterator<char>());
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };

  ros::Publisher mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);
  ros::Publisher robot_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster;

  auto mesh_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/meshes/dining_chair.ply";
  auto point_cloud_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/point_clouds/dining_chair.pcd";

  Model model;
  model.load(mesh_filename, point_cloud_filename);

  GripperParameters params{};
  params.gripper_description_ = loadFileContent("/home/philipp/chair_manipulation_ws/src/chair_manipulation/chair_manipulation_grasp_detection_advanced/cfg/gripper/robotiq_2f_140.urdf");
  params.gripper_semantic_description_ = loadFileContent("/home/philipp/chair_manipulation_ws/src/chair_manipulation/chair_manipulation_grasp_detection_advanced/cfg/gripper/robotiq_2f_140.srdf");
  params.base_frame_ = "robotiq_arg2f_base_link";
  params.tcp_frame_ = "tcp";
  FingerGroup left_finger_group;
  left_finger_group.group_name_ = "left_finger";
  left_finger_group.open_group_state_name_ = "left_finger_open";
  left_finger_group.closed_group_state_name_ = "left_finger_closed";
  FingerGroup right_finger_group;
  right_finger_group.group_name_ = "right_finger";
  right_finger_group.open_group_state_name_ = "right_finger_open";
  right_finger_group.closed_group_state_name_ = "right_finger_closed";
  params.finger_groups_ = { left_finger_group, right_finger_group };

  Eigen::Isometry3d tcp_pose = Eigen::Isometry3d::Identity();

  Gripper gripper{ params };
  gripper.setTcpPose(tcp_pose);
  gripper.setStateOpen();
  gripper.addCollisionObject(model.mesh_);

  bool colliding = gripper.isColliding();
  ROS_INFO_STREAM("colliding: " << colliding);

  std::vector<Contact> contacts;
  bool success = gripper.grasp(contacts);
  ROS_INFO_STREAM("grasp success: " << success);

  geometry_msgs::TransformStamped msg = tf2::eigenToTransform(gripper.getBasePose());
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "parent_world";
  msg.child_frame_id = "world";

  broadcaster.sendTransform(msg);

  for (std::size_t i = 0; i < contacts.size(); i++)
  {
    const auto& contact = contacts[i];
    Eigen::Isometry3d T;
    T = Eigen::Translation3d{contact.position_} * transform::fromYAxis(contact.normal_);
    msg = tf2::eigenToTransform(T);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "parent_world";
    msg.child_frame_id = "contact" + std::to_string(i);
    broadcaster.sendTransform(msg);
  }

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::convert(*model.mesh_, mesh_msg);

  auto robot_state = gripper.getRobotState();
  moveit_msgs::DisplayRobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(*robot_state, robot_state_msg.state);
  robot_state_msg.hide = false;

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    mesh_pub.publish(mesh_msg);
    robot_state_pub.publish(robot_state_msg);
    rate.sleep();
  }

  return 0;
}