#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H

#include <Eigen/src/Core/Matrix.h>
namespace chair_manipulation
{
struct Contact
{
  Contact() = default;

  Contact(const Eigen::Vector3d& position, const Eigen::Vector3d& normal) : position_(position), normal_(normal)
  {
  }

  Eigen::Vector3d position_;
  Eigen::Vector3d normal_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_CONTACT_H
