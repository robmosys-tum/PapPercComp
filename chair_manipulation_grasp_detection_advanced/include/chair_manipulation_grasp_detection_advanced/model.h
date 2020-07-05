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
  using PointCloud = pcl::PointCloud<pcl::PointNormal>;

  void load(const std::string& mesh_filename, const std::string& point_cloud_filename);

  Mesh mesh_;
  PointCloud point_cloud_;
};

class ModelException : std::runtime_error
{
public:
  explicit ModelException(const std::string& msg) : std::runtime_error(msg)
  {
  }
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_MODEL_H
