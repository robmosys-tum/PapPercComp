#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H

#include <pcl/PolygonMesh.h>
#include <geometric_shapes/shapes.h>
#include <shape_msgs/Mesh.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

namespace chair_manipulation
{
namespace utils
{
void convert(const pcl::PolygonMesh& from, shapes::Mesh& mesh);

void convert(const shapes::Mesh& from, shape_msgs::Mesh& to);

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
Vector3<T> projection(const Vector3<T>& axis, const Vector3<T>& v)
{
  Vector3<T> axis_normalized = axis.normalized();
  return v.dot(axis_normalized) * axis_normalized;
}

template <typename PointT>
void publishPointCloud(const pcl::PointCloud<PointT>& pcl_cloud, ros::Publisher& publisher, const std::string& frame)
{
  sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(pcl_cloud, *pc2_cloud);
  pc2_cloud->header.frame_id = frame;
  pc2_cloud->header.stamp = ros::Time::now();
  publisher.publish(pc2_cloud);
}

std::string loadStringParameter(const XmlRpc::XmlRpcValue& value, const std::string& key);

double loadDoubleParameter(const XmlRpc::XmlRpcValue& value, const std::string& key);

}  // namespace utils
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
