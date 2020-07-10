#include <chair_manipulation_grasp_detection_advanced/utils.h>
#include "chair_manipulation_grasp_detection_advanced/grasp_database_creator.h"
#include "chair_manipulation_grasp_detection_advanced/exception.h"

namespace chair_manipulation
{

void GraspDatabaseCreatorParameters::load(ros::NodeHandle& nh)
{
  grasp_sampler_params_.load(nh);

  sample_trials_ = nh.param<int>("sample_trails", 1000);

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
  for (const auto& model : params_.models_)
  {
    auto element = std::make_shared<GraspDatabaseElement>();
    element->model_ = model;
    grasp_sampler_.sampleGrasps(model, params_.sample_trials_, element->grasps_);
    database.add(element);
  }
}

}  // namespace chair_manipulation