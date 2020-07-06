#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H

#include "model.h"
#include "grasp.h"

namespace chair_manipulation
{
struct GraspDatabaseElement
{
  Model model_;
  std::vector<Grasp> grasps_;
};

using GraspDatabaseElementPtr = std::shared_ptr<GraspDatabaseElement>;
using GraspDatabaseElementConstPtr = std::shared_ptr<GraspDatabaseElement>;

}

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_GRASP_DATABASE_ELEMENT_H
