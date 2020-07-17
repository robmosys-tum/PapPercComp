#include "test_common.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace chair_manipulation;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "registration_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{ "~" };
  auto source_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_source", 1);
  auto target_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_target", 1);
  auto aligned_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_aligned", 1);
  TestParameters params;

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();

  auto source_cloud = PointCloudPtr{ new PointCloud };
  pcl::copyPointCloud(*params.model_->getPointCloud(), *source_cloud);

  auto target_cloud = PointCloudPtr{ new PointCloud };
  std::string cloud_filename = ros::package::getPath("chair_manipulation_grasp_detection_advanced") + "/test/clouds/"
                                                                                                      "chair_segmented."
                                                                                                      "pcd";

  pcl::io::loadPCDFile(cloud_filename, *target_cloud);

  auto filtered_source_cloud = PointCloudPtr{ new PointCloud };
  auto filtered_target_cloud = PointCloudPtr{ new PointCloud };

  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(0.05, 0.05, 0.05);

  voxel_filter.setInputCloud(source_cloud);
  voxel_filter.filter(*filtered_source_cloud);

  voxel_filter.setInputCloud(target_cloud);
  voxel_filter.filter(*filtered_target_cloud);

  auto source_eigen_cloud = std::make_shared<Eigen::MatrixXd>();
  auto target_eigen_cloud = std::make_shared<Eigen::MatrixXd>();

  utils::pointCloudToEigen(*filtered_source_cloud, *source_eigen_cloud);
  utils::pointCloudToEigen(*filtered_target_cloud, *target_eigen_cloud);

  ROS_INFO_STREAM("Number of points of source cloud: " << source_eigen_cloud->rows());
  ROS_INFO_STREAM("Number of points of target cloud: " << target_eigen_cloud->rows());

  Eigen::MatrixXd aligned_eigen_cloud;
  NonrigidTransform transform;
  PointCloudRegistration registration{ params.point_cloud_registration_params_ };
  registration.setInputSource(source_eigen_cloud);
  registration.setInputTarget(target_eigen_cloud);
  registration.align(aligned_eigen_cloud, transform);

  ROS_INFO_STREAM("Registration finished.");

  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  utils::eigenToPointCloud(aligned_eigen_cloud, aligned_cloud);

  Eigen::Isometry3d original_pose = utils::poseFromStr("0.3743 0.3447 0.43 0 0 0 1");
  Eigen::Isometry3d transformed_pose = transform * original_pose;

  visual_tools.publishArrow(original_pose, rvt::BLUE);
  visual_tools.publishArrow(transformed_pose, rvt::RED);

  visual_tools.trigger();

  ros::Rate rate{ 10 };
  while (ros::ok())
  {
    utils::publishPointCloud(*filtered_source_cloud, source_cloud_pub, "world");
    utils::publishPointCloud(*filtered_target_cloud, target_cloud_pub, "world");
    utils::publishPointCloud(aligned_cloud, aligned_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}