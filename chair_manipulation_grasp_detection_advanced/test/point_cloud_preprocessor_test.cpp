#include "test_common.h"
#include <ros/ros.h>
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"

using namespace chair_manipulation;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "point_cloud_preprocessor_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv{"~"};
  auto point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
  auto preprocessed_point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_preprocessed", 1);
  TestParameters params;

  auto input_point_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr{ new pcl::PointCloud<pcl::PointXYZ> };
  pcl::PointCloud<pcl::PointNormal> preprocessed_point_cloud;

  pcl::copyPointCloud(*params.model_->getPointCloud(), *input_point_cloud);

  PointCloudPreprocessor preprocessor{params.point_cloud_preprocessor_params_};
  preprocessor.setInputCloud(input_point_cloud);
  preprocessor.preprocess(preprocessed_point_cloud);

  ros::Rate rate{10};
  while (ros::ok())
  {
    utils::publishPointCloud(*params.model_->getPointCloud(), point_cloud_pub, "world");
    utils::publishPointCloud(preprocessed_point_cloud, preprocessed_point_cloud_pub, "world");
    rate.sleep();
  }

  return 0;
}