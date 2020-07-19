#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H

#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <ros/ros.h>

namespace chair_manipulation
{
struct MeshReconstructionParameters
{
  void load(ros::NodeHandle& nh);

  double search_radius_;
  double relative_max_distance_;
  int max_nearest_neighbors_;
  double max_surface_angle_;
  double min_angle_;
  double max_angle_;
  bool normal_consistency_;
};

class MeshReconstruction
{
public:
  using PointT = pcl::PointNormal;
  using PointCloud = pcl::PointCloud<PointT>;
  using PointCloudConstPtr = PointCloud::ConstPtr;
  using SearchMethod = pcl::search::KdTree<PointT>;
  using SearchMethodPtr = SearchMethod::Ptr;
  using ReconstructionMethod = pcl::GreedyProjectionTriangulation<PointT>;

  explicit MeshReconstruction(MeshReconstructionParameters params);

  void setInputCloud(const PointCloudConstPtr& input);

  void reconstruct(pcl::PolygonMesh& mesh);

private:
  MeshReconstructionParameters params_;
  ReconstructionMethod reconstruction_method_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MESH_RECONSTRUCTION_H
