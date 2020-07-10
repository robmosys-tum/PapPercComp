#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include <pcl/io/vtk_lib_io.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <shape_msgs/MeshTriangle.h>
#include <geometry_msgs/Point.h>

namespace chair_manipulation
{
namespace utils
{
void convert(const pcl::PolygonMesh& from, shapes::Mesh& to)
{
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(from, polydata);

  // Make sure that the polygons are triangles
  vtkSmartPointer<vtkTriangleFilter> triangle_filter = vtkSmartPointer<vtkTriangleFilter>::New();
  triangle_filter->SetInputData(polydata);
  triangle_filter->Update();

  vtkSmartPointer<vtkPolyDataMapper> triangle_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangle_mapper->SetInputConnection(triangle_filter->GetOutputPort());
  triangle_mapper->Update();
  polydata = triangle_mapper->GetInput();

  polydata->BuildCells();
  auto points = polydata->GetPoints();
  vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

  auto num_vertices = points->GetNumberOfPoints();
  auto num_triangles = cells->GetNumberOfCells();

  // The shapes::Mesh class does not have an assignment operator (which is a shame) so we have to
  // do the memory management by ourselves...
  if (to.vertices)
    delete[] to.vertices;
  if (to.vertex_normals)
    delete[] to.vertex_normals;
  if (to.triangles)
    delete[] to.triangles;
  if (to.triangle_normals)
    delete[] to.triangle_normals;
  to.vertex_count = num_vertices;
  to.triangle_count = num_triangles;
  to.vertices = new double[num_vertices * 3];
  to.vertex_normals = new double[num_vertices * 3];
  to.triangles = new unsigned int[num_triangles * 3];
  to.triangle_normals = new double[num_triangles * 3];

  // Copy vertices
  for (std::size_t i = 0; i < num_vertices; i++)
  {
    points->GetPoint(i, to.vertices + i * 3);
  }

  // Copy triangle indices
  vtkIdType num_points = 0;
  vtkIdType* point_ids = nullptr;
  std::size_t cell_id = 0;
  for (cells->InitTraversal(); cells->GetNextCell(num_points, point_ids); cell_id++)
  {
    to.triangles[cell_id * 3 + 0] = point_ids[0];
    to.triangles[cell_id * 3 + 1] = point_ids[1];
    to.triangles[cell_id * 3 + 2] = point_ids[2];
  }

  // Compute normals
  to.computeTriangleNormals();
  to.computeVertexNormals();
}

void convert(const shapes::Mesh& from, shape_msgs::Mesh& to)
{
  // Copy vertices
  for (std::size_t i = 0; i < from.vertex_count; i++)
  {
    geometry_msgs::Point point;
    point.x = from.vertices[i * 3 + 0];
    point.y = from.vertices[i * 3 + 1];
    point.z = from.vertices[i * 3 + 2];
    to.vertices.push_back(point);
  }

  // Copy triangles
  for (std::size_t i = 0; i < from.triangle_count; i++)
  {
    shape_msgs::MeshTriangle triangle;
    triangle.vertex_indices[0] = from.triangles[i * 3 + 0];
    triangle.vertex_indices[1] = from.triangles[i * 3 + 1];
    triangle.vertex_indices[2] = from.triangles[i * 3 + 2];
    to.triangles.push_back(triangle);
  }
}

std::string loadStringParameter(const XmlRpc::XmlRpcValue& value, const std::string& key)
{
  if (!value.hasMember(key))
  {
    std::ostringstream msg;
    msg << "Attribute '" << key << "' not found.";
    throw exception::Parameter{ msg.str() };
  }
  XmlRpc::XmlRpcValue attribute = value[key];
  if (attribute.getType() != XmlRpc::XmlRpcValue::TypeString)
  {
    std::ostringstream msg;
    msg << "Attribute '" << key << "' must be of type string.";
    throw exception::Parameter{ msg.str() };
  }
  return (std::string) attribute;
}

}  // namespace utils
}  // namespace chair_manipulation