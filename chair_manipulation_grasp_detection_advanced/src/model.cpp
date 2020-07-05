#include "chair_manipulation_grasp_detection_advanced/model.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>

namespace chair_manipulation
{
void Model::load(const std::string& mesh_filename, const std::string& point_cloud_filename)
{
  if (!pcl::io::loadPCDFile(point_cloud_filename, point_cloud_))
  {
    throw ModelException{ "Failed to load model point cloud." };
  }

  pcl::PolygonMesh mesh;
  if (!pcl::io::loadPolygonFilePLY(mesh_filename, mesh))
  {
    throw ModelException{"Failed to load mesh."};
  }
  utils::convert(mesh, mesh_);
}

}  // namespace chair_manipulation