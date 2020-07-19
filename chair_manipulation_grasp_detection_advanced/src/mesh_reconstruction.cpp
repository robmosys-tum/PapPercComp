#include <chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h>

namespace chair_manipulation
{
void MeshReconstructionParameters::load(ros::NodeHandle& nh)
{
  search_radius_ = nh.param<double>("search_radius", 0.025);
  relative_max_distance_ = nh.param<double>("relative_max_distance", 2.5);
  max_nearest_neighbors_ = nh.param<int>("max_nearest_neighbors", 100);
  max_surface_angle_ = nh.param<double>("max_surface_angle", M_PI / 4);
  min_angle_ = nh.param<double>("min_angle", M_PI / 18);
  max_angle_ = nh.param<double>("max_angle", 2 * M_PI / 3);
  normal_consistency_ = nh.param<bool>("normal_consistency", false);
}

MeshReconstruction::MeshReconstruction(MeshReconstructionParameters params) : params_(std::move(params))
{
  auto search_method = SearchMethodPtr{ new SearchMethod };
  reconstruction_method_.setSearchMethod(search_method);
  reconstruction_method_.setSearchRadius(params_.search_radius_);
  reconstruction_method_.setMu(params_.relative_max_distance_);
  reconstruction_method_.setMaximumNearestNeighbors(params_.max_nearest_neighbors_);
  reconstruction_method_.setMaximumSurfaceAngle(params_.max_surface_angle_);
  reconstruction_method_.setMinimumAngle(params_.min_angle_);
  reconstruction_method_.setMaximumAngle(params_.max_angle_);
  reconstruction_method_.setNormalConsistency(params_.normal_consistency_);
}

void MeshReconstruction::setInputCloud(const PointCloudConstPtr& input)
{
  reconstruction_method_.setInputCloud(input);
}

void MeshReconstruction::reconstruct(pcl::PolygonMesh& mesh)
{
  reconstruction_method_.reconstruct(mesh);
}

}  // namespace chair_manipulation
