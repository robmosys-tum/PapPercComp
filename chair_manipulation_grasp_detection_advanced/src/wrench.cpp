#include "chair_manipulation_grasp_detection_advanced/wrench.h"

namespace chair_manipulation
{
WrenchSpace::WrenchSpace(const std::vector<Contact>& contacts, double friction_coefficient,
                         std::size_t num_friction_edges)
  : friction_coefficient_(friction_coefficient), num_friction_edges_(num_friction_edges)
{
  for (const auto& contact : contacts)
    addContactWrenches(contact);

  computeConvexHull();
  computeEpsilon1Quality();
  computeV1Quality();
}

void WrenchSpace::addContactWrenches(const Contact& contact)
{

}

void WrenchSpace::computeConvexHull()
{

}

void WrenchSpace::computeEpsilon1Quality()
{

}

void WrenchSpace::computeV1Quality()
{

}

}  // namespace chair_manipulation
