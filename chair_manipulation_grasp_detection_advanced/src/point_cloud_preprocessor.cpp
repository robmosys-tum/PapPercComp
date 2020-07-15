#include <chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h>
#include <pcl/features/normal_3d.h>

namespace chair_manipulation
{
void PointCloudPreprocessorParameters::load(ros::NodeHandle& nh)
{
  voxel_leaf_size_ = nh.param<double>("voxel_leaf_size", 0.01);
  mean_k_ = nh.param<double>("mean_k", 50);
  stddev_mul_threshold_ = nh.param<double>("stddev_mul_threshold", 1.);
  normal_search_radius_ = nh.param<double>("normal_search_radius", 0.05);
}

PointCloudPreprocessor::PointCloudPreprocessor(PointCloudPreprocessorParameters params) : params_(std::move(params))
{
  search_method_ = SearchMethodPtr{ new SearchMethod };
  voxel_filter_.setLeafSize(params_.voxel_leaf_size_, params_.voxel_leaf_size_, params_.voxel_leaf_size_);
  outlier_filter_.setMeanK(params_.mean_k_);
  outlier_filter_.setStddevMulThresh(params_.stddev_mul_threshold_);
  normal_estimation_.setRadiusSearch(params_.normal_search_radius_);
  normal_estimation_.setSearchMethod(search_method_);
}

void PointCloudPreprocessor::setInputCloud(const PointCloudPreprocessor::InputPointCloudConstPtr& input)
{
  input_ = input;
}

void PointCloudPreprocessor::preprocess(OutputPointCloud& output)
{
  auto voxel_filtered = InputPointCloudPtr{ new InputPointCloud };
  auto outlier_filtered = InputPointCloudPtr{ new InputPointCloud };
  auto normals = SurfaceNormalsPtr{ new SurfaceNormals };

  voxel_filter_.setInputCloud(input_);
  voxel_filter_.filter(*voxel_filtered);

  outlier_filter_.setInputCloud(voxel_filtered);
  outlier_filter_.filter(*outlier_filtered);

  normal_estimation_.setInputCloud(outlier_filtered);
  normal_estimation_.compute(*normals);

  pcl::concatenateFields(*outlier_filtered, *normals, output);
}

}  // namespace chair_manipulation
