#include "test_common.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner{ 1 };
  tf2_ros::StaticTransformBroadcaster broadcaster;
  geometry_msgs::TransformStamped msg;
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);

  TestParameters params;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  auto gripper1 = std::make_shared<Gripper>(params.gripper_params_, params.gripper_urdf_, params.gripper_srdf_);
  auto gripper2 = std::make_shared<Gripper>(params.gripper_params_, params.gripper_urdf_, params.gripper_srdf_);

  auto sampler1 = std::make_shared<GraspSampler>(params.grasp_sampler_params_, gripper1);
  auto sampler2 = std::make_shared<GraspSampler>(params.grasp_sampler_params_, gripper2);

  int index1 = 12000;
  int index2 = 3200;

  const auto& point1 = params.model_->getPointCloud()->at(index1);
  const auto& point2 = params.model_->getPointCloud()->at(index2);

  Eigen::Isometry3d pose1, pose2;

  bool found_pose1 = sampler1->findGraspPoseAt(params.model_->getPointCloud(), point1, pose1);
  bool found_pose2 = sampler2->findGraspPoseAt(params.model_->getPointCloud(), point2, pose2);

  ROS_INFO_STREAM("found pose1: " << found_pose1);
  ROS_INFO_STREAM("found pose2: " << found_pose2);

  gripper1->addCollisionObject(params.model_->getMesh());
  gripper2->addCollisionObject(params.model_->getMesh());

  gripper1->setTcpPose(pose1);
  gripper2->setTcpPose(pose2);

  gripper1->setStateOpen();
  gripper2->setStateOpen();

  std::vector<Contact> contacts1, contacts2;

  bool success1 = gripper1->grasp(contacts1);
  bool success2 = gripper2->grasp(contacts2);

  ROS_INFO_STREAM("grasp1 success: " << success1);
  ROS_INFO_STREAM("grasp2 success: " << success2);

  for (const auto& contact : contacts1)
    visual_tools.publishArrow(Eigen::Translation3d{contact.position_} * transform::fromXAxis(contact.normal_), rvt::RED, rvt::SMALL);

  for (const auto& contact : contacts2)
    visual_tools.publishArrow(Eigen::Translation3d{contact.position_} * transform::fromXAxis(contact.normal_), rvt::RED, rvt::SMALL);

  std::vector<Contact> contacts;
  std::copy(contacts1.begin(), contacts1.end(), std::back_inserter(contacts));
  std::copy(contacts2.begin(), contacts2.end(), std::back_inserter(contacts));

  WrenchSpace wrench_space{contacts, *params.model_, 0.8, 8};
  for (const auto& wrench : wrench_space.getWrenches())
  {
    visual_tools.publishArrow(transform::fromXAxis(wrench.getForce()), rvt::GREEN);
    visual_tools.publishArrow(transform::fromXAxis(wrench.getTorque()), rvt::BLUE);
  }

  ROS_INFO_STREAM("force closure: " << wrench_space.isForceClosure());
  ROS_INFO_STREAM("epsilon1 quality: " << wrench_space.getEpsilon1Quality());
  ROS_INFO_STREAM("v1 quality: " << wrench_space.getV1Quality());

  visual_tools.trigger();

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*params.model_->getPointCloud(), point_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}