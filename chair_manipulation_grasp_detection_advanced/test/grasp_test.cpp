#include "test_common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  auto mesh_pub = nh.advertise<shape_msgs::Mesh>("mesh", 1);
  auto robot_state_pub = nh.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped msg;
  TestParameters params;
  auto gripper = std::make_shared<Gripper>(params.gripper_params_, params.gripper_urdf_, params.gripper_srdf_);

  Eigen::Isometry3d tcp_pose = Eigen::Isometry3d::Identity();
  gripper->setTcpPose(tcp_pose);
  gripper->setStateOpen();
  gripper->addCollisionObject(params.model_->getMesh());

  bool colliding = gripper->isColliding();
  ROS_INFO_STREAM("colliding: " << colliding);

  std::vector<Contact> contacts;
  bool success = gripper->grasp(contacts);
  ROS_INFO_STREAM("grasp success: " << success);

  msg = tf2::eigenToTransform(gripper->getBasePose());
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "parent_world";
  msg.child_frame_id = "world";

  broadcaster.sendTransform(msg);

  for (std::size_t i = 0; i < contacts.size(); i++)
  {
    const auto& contact = contacts[i];
    Eigen::Isometry3d T;
    T = Eigen::Translation3d{ contact.position_ } * transform::fromYAxis(contact.normal_);
    msg = tf2::eigenToTransform(T);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "parent_world";
    msg.child_frame_id = "contact" + std::to_string(i);
    broadcaster.sendTransform(msg);
  }

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::convert(*params.model_->getMesh(), mesh_msg);

  auto robot_state = gripper->getRobotState();
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