#include "chair_manipulation_grasp_detection_advanced/model.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <memory>

namespace chair_manipulation
{
void Model::load(const std::string& mesh_filename, const std::string& point_cloud_filename)
{
  // Reset pointers
  mesh_ = std::make_shared<Mesh>();
  point_cloud_ = PointCloudPtr{ new PointCloud };

  // Load mesh
  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFilePLY(mesh_filename, mesh))
  {
    std::ostringstream msg;
    msg << "Failed to load model mesh '" << mesh_filename << "'.";
    throw exception::IO{ msg.str() };
  }
  utils::convert(mesh, *mesh_);

  // Load point cloud
  if (pcl::io::loadPCDFile(point_cloud_filename, *point_cloud_) != 0)
  {
    std::ostringstream msg;
    msg << "Failed to load model point cloud '" << point_cloud_filename << "'.";
    throw exception::IO{ msg.str() };
  }
}

}  // namespace chair_manipulation