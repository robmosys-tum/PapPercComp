#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

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
    auto mesh_filename = utils::loadStringParameter(model_item, "mesh");
    auto point_cloud_filename = utils::loadStringParameter(model_item, "point_cloud");
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