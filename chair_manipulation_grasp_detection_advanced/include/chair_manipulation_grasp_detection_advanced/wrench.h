#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H

#include "contact.h"
#include <vector>

namespace chair_manipulation
{
struct Wrench
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Wrench() = default;

  Wrench(const Eigen::Vector3d& force, const Eigen::Vector3d& torque) : force_(force), torque_(torque)
  {
  }

  Eigen::Vector3d force_;
  Eigen::Vector3d torque_;
};

struct Hyperplane
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Vector6d = Eigen::Matrix<double, 6, 1>;

  Hyperplane() = default;

  Hyperplane(const Vector6d& coeffs, double offset) : coeffs_(coeffs), offset_(offset)
  {
  }

  Vector6d coeffs_;
  double offset_;
};

class WrenchSpace
{
public:
  explicit WrenchSpace(const std::vector<Contact>& contacts, double friction_coefficient,
                       std::size_t num_friction_edges);

  const std::vector<Wrench>& getWrenches() const
  {
    return wrenches;
  }

  double getFrictionCoefficient() const
  {
    return friction_coefficient_;
  }

  std::size_t getNumFrictionEdges() const
  {
    return num_friction_edges_;
  }

  const std::vector<Hyperplane>& getHyperplanes() const
  {
    return hyperplanes_;
  }

  bool isForceClosure() const
  {
    return force_closure_;
  }

  double getEpsilon1Quality() const
  {
    return epsilon1_quality_;
  }

  double getV1Quality() const
  {
    return v1_quality_;
  }

private:
  std::vector<Wrench> wrenches;
  double friction_coefficient_;
  std::size_t num_friction_edges_;
  std::vector<Hyperplane> hyperplanes_;
  bool force_closure_;
  double epsilon1_quality_;
  double v1_quality_;

  void addContactWrenches(const Contact& contact);

  void computeConvexHull();

  void computeEpsilon1Quality();

  void computeV1Quality();
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_WRENCH_H
