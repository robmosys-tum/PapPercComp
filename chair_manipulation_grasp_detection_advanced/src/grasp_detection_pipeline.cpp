#include "chair_manipulation_grasp_detection_advanced/grasp_detection_pipeline.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_preprocessor.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_matcher.h"
#include "chair_manipulation_grasp_detection_advanced/point_cloud_registration.h"
#include "chair_manipulation_grasp_detection_advanced/gripper.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_sampler.h"
#include "chair_manipulation_grasp_detection_advanced/grasp_synthesizer.h"
#include "chair_manipulation_grasp_detection_advanced/mesh_reconstruction.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"
#include "chair_manipulation_grasp_detection_advanced/stopwatch.h"
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <iostream>

namespace chair_manipulation
{
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;
using PointNormalCloud = pcl::PointCloud<pcl::PointNormal>;
using PointNormalCloudPtr = PointNormalCloud::Ptr;
using EigenCloud = Eigen::MatrixXd;
using EigenCloudPtr = std::shared_ptr<EigenCloud>;
using PolygonMesh = pcl::PolygonMesh;
using PolygonMeshPtr = PolygonMesh::Ptr;
using ShapeMesh = shapes::Mesh;
using ShapeMeshPtr = std::shared_ptr<ShapeMesh>;

void GraspDetectionParameters::load(ros::NodeHandle& nh)
{
  num_sample_trials_ = nh.param<int>("num_sample_trials", 100);

  XmlRpc::XmlRpcValue grasp_frame_items;
  if (!nh.getParam("grasp_frames", grasp_frame_items) ||
      grasp_frame_items.getType() != XmlRpc::XmlRpcValue::TypeArray || grasp_frame_items.size() == 0)
    throw exception::Parameter{ "Failed to load parameter 'topics'." };

  int num_grasp_frames = grasp_frame_items.size();
  grasp_frames_.resize(num_grasp_frames);
  for (int i = 0; i < num_grasp_frames; i++)
  {
    XmlRpc::XmlRpcValue grasp_frame_item = grasp_frame_items[i];
    grasp_frames_[i] = (std::string)grasp_frame_item;
  }
}

void transformGrasps(const std::vector<MultiArmGrasp>& source_grasps, std::vector<MultiArmGrasp>& target_grasps, const NonrigidTransform& transform)
{
  target_grasps.resize(source_grasps.size());
  for (std::size_t i = 0; i < source_grasps.size(); i++)
  {
    const auto& source_grasp = source_grasps[i];
    auto& target_grasp = target_grasps[i];
    target_grasp.poses_.resize(source_grasp.poses_.size());
    for (std::size_t j = 0; j < source_grasp.poses_.size(); j++)
      target_grasp.poses_[j] = transform * source_grasp.poses_[j];
  }
}

void run_grasp_detection_pipeline()
{
  try
  {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv{ "~" };
    std::string gripper_urdf, gripper_srdf, segmented_cloud_topic;
    if (!nh_priv.getParam("gripper_urdf", gripper_urdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_urdf'." };
    if (!nh_priv.getParam("gripper_srdf", gripper_srdf))
      throw exception::Parameter{ "Failed to load parameter 'gripper_srdf'." };
    if (!nh_priv.getParam("segmented_cloud_topic", segmented_cloud_topic))
      throw exception::Parameter{ "Failed to load parameter 'segmented_cloud_topic'." };

    GraspDetectionParameters grasp_detection_params;
    ros::NodeHandle grasp_detection_nh{ "~grasp_detection" };
    grasp_detection_params.load(grasp_detection_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Creating gripper.");
    GripperParameters gripper_params;
    ros::NodeHandle gripper_nh{ "~gripper" };
    gripper_params.load(gripper_nh);
    auto gripper = std::make_shared<Gripper>(std::move(gripper_params), gripper_urdf, gripper_srdf);

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp sampler.");
    GraspSamplerParameters grasp_sampler_params;
    ros::NodeHandle grasp_sampler_nh{ "~grasp_sampler" };
    grasp_sampler_params.load(grasp_sampler_nh);
    GraspSampler grasp_sampler{ std::move(grasp_sampler_params), gripper };

    ROS_DEBUG_STREAM_NAMED("main", "Creating grasp synthesizer.");
    GraspQualityWeights grasp_quality_weights;
    ros::NodeHandle grasp_quality_weights_nh{ "~weights_online" };
    grasp_quality_weights.load(grasp_quality_weights_nh);

    GraspSynthesizerParameters grasp_synthesizer_params;
    ros::NodeHandle grasp_synthesizer_nh{ "~grasp_synthesizer" };
    grasp_synthesizer_params.load(grasp_synthesizer_nh);
    auto world_frame = grasp_synthesizer_params.world_frame_;
    GraspSynthesizer grasp_synthesizer{ std::move(grasp_synthesizer_params), std::move(grasp_quality_weights) };

    ROS_DEBUG_STREAM_NAMED("main", "Loading grasp database.");
    auto grasp_database = std::make_shared<GraspDatabase>();
    ros::NodeHandle grasp_database_nh{ "~grasp_database" };
    grasp_database->load(grasp_database_nh);

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud preprocessor.");
    PointCloudPreprocessorParameters preprocessor_params;
    ros::NodeHandle preprocessor_nh{ "~point_cloud_preprocessor" };
    preprocessor_params.load(preprocessor_nh);
    PointCloudPreprocessor preprocessor{ std::move(preprocessor_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud matcher.");
    PointCloudMatcherParameters point_cloud_matcher_params;
    ros::NodeHandle point_cloud_matcher_nh{ "~point_cloud_matcher" };
    point_cloud_matcher_params.load(point_cloud_matcher_nh);
    PointCloudMatcher point_cloud_matcher{ std::move(point_cloud_matcher_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating point cloud registration.");
    PointCloudRegistrationParameters point_cloud_registration_params;
    ros::NodeHandle point_cloud_registration_nh{ "~point_cloud_registration" };
    point_cloud_registration_params.load(point_cloud_registration_nh);
    PointCloudRegistration point_cloud_registration{ std::move(point_cloud_registration_params) };

    ROS_DEBUG_STREAM_NAMED("main", "Creating mesh reconstruction.");
    MeshReconstructionParameters mesh_reconstruction_params;
    ros::NodeHandle mesh_reconstruction_nh{ "mesh_reconstruction" };
    MeshReconstruction mesh_reconstruction{ std::move(mesh_reconstruction_params) };

    tf2_ros::TransformBroadcaster broadcaster;
    Stopwatch stopwatch_iteration, stopwatch_step;

    while (ros::ok())
    {
      ROS_DEBUG_STREAM_NAMED("main", "Start next iteration.");
      stopwatch_iteration.start();

      auto segmented_point_cloud2 = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(segmented_cloud_topic, nh);
      auto segmented_point_cloud = PointNormalCloudPtr{ new PointNormalCloud };
      pcl::fromROSMsg(*segmented_point_cloud2, *segmented_point_cloud);

      stopwatch_step.start();
      GraspDatabaseElementConstPtr matched_element;
      point_cloud_matcher.setInputCloud(segmented_point_cloud);
      point_cloud_matcher.setInputDatabase(grasp_database);
      point_cloud_matcher.match(matched_element);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Matching finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      auto source_eigen_cloud = std::make_shared<EigenCloud>();
      auto target_eigen_cloud = std::make_shared<EigenCloud>();
      utils::pointCloudToEigen(*matched_element->model_->getPointCloud(), *source_eigen_cloud);
      utils::pointCloudToEigen(*segmented_point_cloud, *target_eigen_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Conversion from point cloud to eigen cloud finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      EigenCloud aligned_eigen_cloud;
      NonrigidTransform nonrigid_transform;
      point_cloud_registration.setInputSource(source_eigen_cloud);
      point_cloud_registration.setInputTarget(target_eigen_cloud);
      point_cloud_registration.align(aligned_eigen_cloud, nonrigid_transform);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Registration finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      const auto& matched_grasps = matched_element->grasps_;
      std::vector<MultiArmGrasp> prior_grasps;
      transformGrasps(matched_grasps, prior_grasps, nonrigid_transform);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Transforming grasps finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      auto aligned_cloud = PointCloudPtr{ new PointCloud };
      utils::eigenToPointCloud(aligned_eigen_cloud, *aligned_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Conversion from eigen cloud to point cloud finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      auto aligned_normal_cloud = PointNormalCloudPtr{ new PointNormalCloud };
      preprocessor.setInputCloud(aligned_cloud);
      preprocessor.estimateNormals(*aligned_normal_cloud);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Normal estimation finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      PolygonMesh polygon_mesh;
      mesh_reconstruction.setInputCloud(aligned_normal_cloud);
      mesh_reconstruction.reconstruct(polygon_mesh);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Mesh reconstruction finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      auto shape_mesh = std::make_shared<ShapeMesh>();
      utils::polygonToShapeMesh(polygon_mesh, *shape_mesh);
      Model model{ shape_mesh, aligned_normal_cloud };
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Creating model finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      std::vector<GraspHypothesis> hypotheses;
      grasp_sampler.sampleGraspHypothesesFromPrior(model, prior_grasps, grasp_detection_params.num_sample_trials_,
                                                   hypotheses);
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Sampling grasp hypotheses finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      std::vector<MultiArmGrasp> grasps;
      grasp_synthesizer.synthesize(hypotheses, model, 1, grasps);
      if (grasps.empty())
      {
        ROS_WARN_STREAM_NAMED("main", "Failed to find a valid grasp.");
        continue;
      }
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Synthesizing grasps finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_step.start();
      const auto& published_grasp = grasps[0];
      for (std::size_t i = 0; i < published_grasp.poses_.size(); i++)
      {
        const auto& pose = published_grasp.poses_[i];
        const auto& frame = grasp_detection_params.grasp_frames_[i];
        auto msg = tf2::eigenToTransform(pose);
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = world_frame;
        msg.child_frame_id = frame;
        broadcaster.sendTransform(msg);
      }
      stopwatch_step.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Publishing grasp transforms finished.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_step.elapsedSeconds() << "s.");

      stopwatch_iteration.stop();
      ROS_DEBUG_STREAM_NAMED("main", "Finished current iteration.");
      ROS_DEBUG_STREAM_NAMED("main", "It took " << stopwatch_iteration.elapsedSeconds() << "s.");
    }
  }
  catch (const exception::Runtime& e)
  {
    // Use std::cerr because rosconsole doesn't seem to work consistently here...
    std::cerr << "ERROR: " << e.what();
  }
}

}  // namespace chair_manipulation