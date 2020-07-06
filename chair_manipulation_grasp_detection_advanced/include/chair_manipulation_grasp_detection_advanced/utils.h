//
// Created by philipp on 04.07.20.
//

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

template <typename T>
void frameOrientation(const Vector3<T>& x_direction, const Vector3<T>& y_direction, Eigen::Quaternion<T>& q)
{
  Vector3<T> original_x_direction = Vector3<T>::UnitX();
  Vector3<T> x_rotation_axis = original_x_direction.cross(x_direction);
  T x_rotation_angle = std::acos(original_x_direction.dot(x_direction));
  q = Eigen::AngleAxis<T>{ x_rotation_angle, x_rotation_axis };

  Vector3<T> original_y_direction = Vector3<T>::UnitY();
  Vector3<T> rotated_original_y_direction = q * original_y_direction;
  Vector3<T> y_rotation_axis = rotated_original_y_direction.cross(y_direction);
  T y_rotation_angle = std::acos(rotated_original_y_direction.dot(y_direction));
  q = Eigen::AngleAxis<T>{ y_rotation_angle, y_rotation_axis } * q;

  q.normalize();
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

}  // namespace utils
}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_UTILS_H
