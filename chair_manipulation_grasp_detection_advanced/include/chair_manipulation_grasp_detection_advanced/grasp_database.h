#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H

#include "grasp_database_element.h"

namespace chair_manipulation
{
class GraspDatabase
{
public:
  void load(const std::string& filename);

  void save(const std::string& filename);

  void add(const GraspDatabaseElementConstPtr &element)
  {
    elements_.push_back(element);
  }

private:
  std::vector<GraspDatabaseElementPtr> elements_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_H
