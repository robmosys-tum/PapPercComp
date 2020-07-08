#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/model.h"
#include "chair_manipulation_grasp_detection_advanced/collision_detection.h"

using namespace chair_manipulation;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{"~"};
  ros::Publisher mesh_pub = nh.advertise<shape_msgs::Mesh>("chair_mesh", 1);

  auto mesh_filename = nh_priv.param<std::string>("mesh_filename", "");
  auto point_cloud_filename = nh_priv.param<std::string>("point_cloud_filename", "");

  Model model;
  model.load(mesh_filename, point_cloud_filename);

  geometry_msgs::Pose gripper_pose;

  CollisionDetection detection{nh_priv};
  detection.checkCollision(model, gripper_pose);

  shape_msgs::Mesh mesh_msg;
  chair_manipulation::utils::convert(*model.mesh_, mesh_msg);

  ros::Rate rate{10};
  while (ros::ok())
  {
    mesh_pub.publish(mesh_msg);
    rate.sleep();
  }

  return 0;
}