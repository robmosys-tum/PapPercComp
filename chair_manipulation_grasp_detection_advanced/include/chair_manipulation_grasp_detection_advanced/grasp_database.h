#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H

#include "grasp_database_element.h"
#include <ros/ros.h>

namespace chair_manipulation
{
struct GraspDatabase
{
  void load(ros::NodeHandle& nh);

  void store(ros::NodeHandle& nh) const;

  std::vector<GraspDatabaseElementPtr> elements_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H
