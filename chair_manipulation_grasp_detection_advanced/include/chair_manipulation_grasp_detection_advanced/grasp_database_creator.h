#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H

#include "model.h"
#include "grasp_database.h"
#include "grasp_sampler.h"
#include "ros/ros.h"
#include <random>
#include <pcl/search/kdtree.h>

namespace chair_manipulation
{
struct GraspDatabaseCreatorParameters
{
  void load(ros::NodeHandle& nh);

  GraspSamplerParameters grasp_sampler_params_;
  int sample_trials_;
  std::vector<Model> models_;
};

class GraspDatabaseCreator
{
public:
  explicit GraspDatabaseCreator(const GraspDatabaseCreatorParameters& params)
      : params_(params), grasp_sampler_(params.grasp_sampler_params_)
  {
  }

  void createGraspDatabase(GraspDatabase& database);

private:
  GraspDatabaseCreatorParameters params_;
  GraspSampler grasp_sampler_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H
