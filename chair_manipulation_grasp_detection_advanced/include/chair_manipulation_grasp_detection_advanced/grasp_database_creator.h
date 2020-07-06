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
class GraspDatabaseCreator
{
public:
  explicit GraspDatabaseCreator(ros::NodeHandle& nh);

  void createGraspDatabase(GraspDatabase& database);

private:
  std::size_t sample_trials;
  std::vector<Model> models_;

  GraspSampler grasp_sampler_;

  void loadModels(ros::NodeHandle& nh);
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_CREATOR_H
