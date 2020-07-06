#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <geometric_shapes/shapes.h>

namespace chair_manipulation
{
struct Model
{
  using Mesh = shapes::Mesh;
  using MeshPtr = std::shared_ptr<Mesh>;
  using PointCloud = pcl::PointCloud<pcl::PointNormal>;
  using PointCloudPtr = PointCloud::Ptr;

  Model() : mesh_(new Mesh), point_cloud_(new PointCloud)
  {
  }

  void load(const std::string& mesh_filename, const std::string& point_cloud_filename);

  MeshPtr mesh_;
  PointCloudPtr point_cloud_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H
