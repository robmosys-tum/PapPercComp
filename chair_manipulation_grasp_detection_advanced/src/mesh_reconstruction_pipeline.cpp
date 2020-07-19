#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction_pipeline.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>

namespace chair_manipulation
{
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;
using PolygonMesh = pcl::PolygonMesh;
using PolygonMeshPtr = PolygonMesh::Ptr;

void runMeshReconstructionPipeline()
{
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{ "~" };
    auto cloud_topic = nh_priv.param<std::string>("segmented_cloud_topic", "cloud");
    auto mesh_topic = nh_priv.param<std::string>("reconstructed_mesh_topic", "mesh");
    auto mesh_pub = nh.advertise<shape_msgs::Mesh>(mesh_topic, 1);

    ROS_DEBUG_STREAM_NAMED("main", "Creating mesh reconstruction.");
    MeshReconstructionParameters mesh_reconstruction_params;
    ros::NodeHandle mesh_reconstruction_nh{ "~mesh_reconstruction" };
    mesh_reconstruction_params.load(mesh_reconstruction_nh);
    MeshReconstruction mesh_reconstruction{ std::move(mesh_reconstruction_params) };

    Stopwatch stopwatch_iteration, stopwatch_step;

    while (ros::ok())
    {
      ROS_DEBUG_STREAM_NAMED("main", "Start next iteration.");
      stopwatch_iteration.start();

      auto point_cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(cloud_topic, nh);
      auto point_cloud = PointNormalCloudPtr{ new PointNormalCloud };
      pcl::fromROSMsg(*point_cloud2, *point_cloud);
      ROS_DEBUG_STREAM_NAMED("main", "Received and converted point cloud.");

      stopwatch_step.start();
      PolygonMesh polygon_mesh;
      mesh_reconstruction.setInputCloud(point_cloud);
      mesh_reconstruction.reconstruct(polygon_mesh);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Mesh reconstruction finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      shape_msgs::Mesh mesh_msg;
      utils::polygonMeshToMsg(polygon_mesh, mesh_msg);
      mesh_pub.publish(mesh_msg);
      ROS_DEBUG_STREAM_NAMED("main", "Published reconstructed mesh.");

      stopwatch_iteration.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Finished current iteration.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_iteration.elapsedSeconds() << "s.");
    }
  }
  catch (const exception::Runtime& e)
  {
    std::cerr << "ERROR: " << e.what();
  }
}

}