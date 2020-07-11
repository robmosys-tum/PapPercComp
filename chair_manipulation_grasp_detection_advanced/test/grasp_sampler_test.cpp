#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

using namespace chair_manipulation;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "convert_mesh_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  tf2_ros::StaticTransformBroadcaster broadcaster;

  auto mesh_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/meshes/"
                                                                                  "dining_chair.ply";
  auto point_cloud_filename = ros::package::getPath("chair_manipulation_chair_models") + "/models/dining_chair/"
                                                                                         "point_clouds/"
                                                                                         "dining_chair.pcd";
  Model model;
  model.load(mesh_filename, point_cloud_filename);

  auto index = 12000;

  GraspSamplerParameters params{};
  params.grasp_quality_threshold_ = 0.7;
  params.max_antipodal_normal_angle_ = 0.1;
  params.max_antipodal_position_angle_ = 0.1;
  params.max_equator_normal_angle_ = 0.1;
  params.gripper_pad_distance_ = 0.1;
  params.gripper_pad_length_ = 0.3;

  GraspSampler sampler{ params };
  auto point_cloud = model.point_cloud_;
  auto point = (*point_cloud)[index];
  Eigen::Isometry3d grasp_pose;
  bool found = sampler.findGraspPoseAt(point_cloud, point, grasp_pose);

  geometry_msgs::TransformStamped msg;

  if (found)
  {
    msg = tf2::eigenToTransform(grasp_pose);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "grasp_pose";
    broadcaster.sendTransform(msg);
  }

  Eigen::Isometry3d point_pose;
  point_pose = Eigen::Translation3d{point.getVector3fMap().cast<double>()} * transform::fromYAxis(point.getNormalVector3fMap().cast<double>());
  msg = tf2::eigenToTransform(point_pose);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.child_frame_id = "point";
  broadcaster.sendTransform(msg);

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*point_cloud, point_cloud_pub, "world");
    rate.sleep();
  }
  return 0;
}