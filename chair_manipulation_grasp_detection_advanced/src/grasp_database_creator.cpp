#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"
#include "chair_manipulation_grasp_detection_advanced/utils.h"

namespace chair_manipulation
{
GraspDatabaseCreator::GraspDatabaseCreator(ros::NodeHandle& nh) : grasp_sampler_(nh)
{
  sample_trials = nh.param<double>("sample_trials", 1000);
  loadModels(nh);
}

void GraspDatabaseCreator::loadModels(ros::NodeHandle& nh)
{
  XmlRpc::XmlRpcValue models_array;
  if (!nh.getParam("models", models_array) || models_array.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      models_array.size() == 0)
  {
    throw exception::Parameter{ "No models specified." };
  }
  int num_models = models_array.size();
  models_.resize(num_models);
  for (int i = 0; i < num_models; i++)
  {
    auto model_item = models_array[i];
    std::string mesh_filename;
    std::string point_cloud_filename;

    // Get mesh filename
    if (!model_item.hasMember("mesh"))
    {
      throw exception::Parameter{ "Attribute 'mesh' not found for model." };
    }
    XmlRpc::XmlRpcValue mesh_item = model_item["mesh"];
    if (mesh_item.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw exception::Parameter{ "Attribute 'mesh' must be of type string." };
    }
    mesh_filename = (std::string)mesh_item;

    // Get point cloud filename
    if (!model_item.hasMember("point_cloud"))
    {
      throw exception::Parameter{ "Attribute 'point_cloud' not found for model." };
    }
    XmlRpc::XmlRpcValue point_cloud_item = model_item["point_cloud"];
    if (point_cloud_item.getType() != XmlRpc::XmlRpcValue::TypeString)
    {
      throw exception::Parameter{ "Attribute 'point_cloud' must be of type string." };
    }
    point_cloud_filename = (std::string)point_cloud_item;

    // Load the model
    models_[i].load(mesh_filename, point_cloud_filename);
  }
}

void GraspDatabaseCreator::createGraspDatabase(GraspDatabase& database)
{
  for (const auto& model : models_)
  {
    auto element = std::make_shared<GraspDatabaseElement>();
    element->model_ = model;
    grasp_sampler_.sampleGrasps(model, sample_trials, element->grasps_);
    database.add(element);
  }
}

}  // namespace chair_manipulation