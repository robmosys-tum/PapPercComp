//
// Created by philipp on 04.07.20.
//

#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H

#include <pcl/PolygonMesh.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/Mesh.h>

namespace chair_manipulation
{
namespace utils
{
void convert(const pcl::PolygonMesh& from, shapes::Mesh& mesh);

void convert(const shapes::Mesh& from, shape_msgs::Mesh& to);
}  // namespace utils
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
