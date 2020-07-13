#include "chair_manipulation_grasp_detection_advanced/wrench.h"
#include "chair_manipulation_grasp_detection_advanced/transform.h"
#include "chair_manipulation_grasp_detection_advanced/qhull_mutex.h"
#include <libqhullcpp/Qhull.h>
#include <libqhullcpp/QhullFacetList.h>
#include <libqhullcpp/QhullFacet.h>
#include <ros/ros.h>

namespace chair_manipulation
{
WrenchSpace::WrenchSpace(const std::vector<Contact>& contacts, const Model& model, double friction_coefficient,
                         std::size_t num_friction_edges)
  : friction_coefficient_(friction_coefficient), num_friction_edges_(num_friction_edges)
{
  for (const auto& contact : contacts)
    addContactWrenches(contact, model);

  computeConvexHull();
}

void WrenchSpace::addContactWrenches(const Contact& contact, const Model& model)
{
  const auto& normal = contact.normal_;
  Eigen::Isometry3d frame = transform::fromZAxis(normal);
  Eigen::Vector3d tangent = frame * Eigen::Vector3d::UnitX();
  Eigen::Vector3d radius = contact.position_ - model.getCenterOfGravity();
  for (std::size_t i = 0; i < num_friction_edges_; i++)
  {
    double angle = i * 2. * M_PI / num_friction_edges_;
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd{ angle, normal };
    Eigen::Vector3d tangent_rotated = q * tangent;
    Eigen::Vector3d force = normal + friction_coefficient_ * tangent_rotated;
    Eigen::Vector3d torque = radius.cross(force) / model.getMaxDistanceToCenterOfGravity();
    wrenches.emplace_back(force, torque);
  }
}

void WrenchSpace::computeConvexHull(std::size_t dim)
{
  QhullLockGuard guard;
  orgQhull::Qhull qhull;

  // Copy wrench data in C-style array in order for qhull to process
  auto num_wrenches = wrenches.size();
  auto wrenches_coords = new coordT[num_wrenches * dim];
  for (std::size_t i = 0; i < num_wrenches; i++)
  {
    const auto& wrench = wrenches[i];
    for (std::size_t j = 0; j < dim; j++)
      wrenches_coords[dim * i + j] = wrench.value_[j];
  }

  // This calls qhull with no input parameters which computes the convex hull
  const char* input_comment = "";
  // n: normals with offset
  // FA: report total area and volume
  const char* qhull_command = "n FA";

  try
  {
    qhull.runQhull(input_comment, dim, num_wrenches, wrenches_coords, qhull_command);

    double max_offset = -std::numeric_limits<double>::max();
    auto facet_list = qhull.facetList();
    for (const auto& facet : facet_list)
    {
      auto qhull_hyperplane = facet.hyperplane();
      if (qhull_hyperplane.isValid())
      {
        auto hyplerplane_coords = qhull_hyperplane.coordinates();

        Hyperplane hyperplane;
        hyperplane.offset_ = qhull_hyperplane.offset();
        for (std::size_t i = 0; i < dim; i++)
          hyperplane.coeffs_[i] = hyplerplane_coords[i];

        hyperplanes_.push_back(hyperplane);
        max_offset = std::max(max_offset, hyperplane.offset_);
      }
    }

    // We have force closure if there doesn't exist a positive offset
    force_closure_ = max_offset <= 0.;
    if (force_closure_)
    {
      // The epsilon1 quality is smallest offset (in magnitude)
      epsilon1_quality_ = -max_offset;
      // The v1 quality is simply the volume of the convex hull
      v1_quality_ = qhull.volume();
    }
  }
  catch (const orgQhull::QhullError& e)
  {
    // Leave default values
  }
  delete[] wrenches_coords;
}

}  // namespace chair_manipulation
