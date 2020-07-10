#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/static_transform_broadcaster.h>

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
  geometry_msgs::Pose grasp_pose;
  bool found = sampler.findGraspPoseAt(point_cloud, point, grasp_pose);

  geometry_msgs::TransformStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  if (found)
  {
    msg.child_frame_id = "grasp_pose";
    msg.transform.translation.x = grasp_pose.position.x;
    msg.transform.translation.y = grasp_pose.position.y;
    msg.transform.translation.z = grasp_pose.position.z;
    msg.transform.rotation.x = grasp_pose.orientation.x;
    msg.transform.rotation.y = grasp_pose.orientation.y;
    msg.transform.rotation.z = grasp_pose.orientation.z;
    msg.transform.rotation.w = grasp_pose.orientation.w;
    broadcaster.sendTransform(msg);
  }

  Eigen::Vector3f y_direction = point.getNormalVector3fMap();
  Eigen::Vector3f random_direction = Eigen::Vector3f::Random();
  auto z_direction = (random_direction - utils::projection(y_direction, random_direction)).normalized();
  Eigen::Quaternionf q = utils::directionsYZToQuaternion(y_direction, z_direction);
  msg.child_frame_id = "point";
  msg.transform.translation.x = point.x;
  msg.transform.translation.y = point.y;
  msg.transform.translation.z = point.z;
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  broadcaster.sendTransform(msg);

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*point_cloud, point_cloud_pub, "world");
    rate.sleep();
  }
  return 0;
}