#ifndef CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H
#define CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H

#include "utils.h"
#include <nanoflann.hpp>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <cpd/utils.hpp>

namespace chair_manipulation
{
class NonrigidTransform
{
public:
  using SearchMethod = nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3>;
  using SearchMethodPtr = std::shared_ptr<SearchMethod>;

  NonrigidTransform(const Eigen::MatrixXd& points, const Eigen::MatrixXd& w, double beta = 1., std::size_t k = 10)
    : points_(points)
    , w_(w)
    , g_(cpd::affinity(points, points, beta))
    , beta_(beta)
    , k_(std::min(k, (std::size_t)points.rows()))
  {
    search_method_ = std::make_shared<SearchMethod>(3, points);
    search_method_->index->buildIndex();
  }

  NonrigidTransform() : NonrigidTransform(Eigen::MatrixXd{ 0, 0 }, Eigen::MatrixXd{ 0, 0 }, 0., 0)
  {
  }

  Eigen::Vector3d operator*(const Eigen::Vector3d& q) const
  {
    // Identity
    if (k_ == 0)
      return q;

    // Remember: T(p_i) = p_i + v(p_i)
    // where p_i is the i-th point in the source cloud and v(p_i) = g_i * w.
    // For a new point q, we have to estimate its transformation by considering its k nearest neighbors.
    // The transformation of q is then a weighted average of the transformation of each q's neighbors
    // which is weighted by the affinity to q.

    // Retrieve k nearest neighbors to the input point
    std::vector<Eigen::MatrixXd::Index> indices(k_);
    std::vector<double> sqr_distances(k_);
    search_method_->query(&q[0], k_, &indices[0], &sqr_distances[0]);

    // Sum of each transformation weighted by affinity
    Eigen::Vector3d v_sum = Eigen::Vector3d::Zero();
    // Sum of affinities
    double v_normalizer = 0.;

    for (std::size_t i = 0; i < k_; i++)
    {
      const Eigen::Vector3d& p_i = points_.row(indices[i]);
      const Eigen::MatrixXd& g_i = g_.row(indices[i]);
      // Compute the affinity between the input point and the current nearest point
      double affinity = cpd::affinity(q.transpose(), p_i, beta_)(0, 0);
      // Compute the transformation of the nearest point
      Eigen::Vector3d v = (g_i * w_).transpose();
      // Weight transform by affinity
      v_sum += affinity * v;
      v_normalizer += affinity;
    }
    return q + (v_sum / v_normalizer);
  }

  Eigen::Isometry3d operator*(const Eigen::Isometry3d& transform) const
  {
    // Transform each basis vector of the rotation matrix individually and orthonormalize
    Eigen::Vector3d t = *this * transform.translation();
    Eigen::Matrix3d r_old = transform.rotation();
    Eigen::Matrix3d r_new;
    r_new.col(0) = *this * r_old.col(0);
    r_new.col(1) = *this * r_old.col(1);
    r_new.col(2) = *this * r_old.col(2);
    Eigen::Quaterniond q;
    q = utils::orthonormalize(r_new);
    q.normalize();
    Eigen::Isometry3d result;
    result = Eigen::Translation3d{ t } * q;
    return result;
  }

private:
  Eigen::MatrixXd points_;
  Eigen::MatrixXd w_;
  Eigen::MatrixXd g_;
  double beta_;
  std::size_t k_;
  std::shared_ptr<SearchMethod> search_method_;
};

}  // namespace chair_manipulation

#endif  // CHAIR_MANIPULATION_GRASP_DETECTION_ADVANCED_NONRIGID_TRANSFORM_H
