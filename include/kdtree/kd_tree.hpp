#pragma once

#include <memory>
#include <vector>

#include "nanoflann.hpp"

namespace kd_tree {
template <typename PointCloudT>
class KdTree {
 public:
  using PointT = typename PointCloudT::PointT;
  using FloatType = typename PointT::FloatType;
  explicit KdTree() {
    params_.sorted = true;
    point_cloud_.reset(new PointCloudT);
  }
  virtual ~KdTree() = default;

 protected:
  std::unique_ptr<PointCloudT> point_cloud_;
  nanoflann::SearchParams params_;

 public:
  const std::vector<PointT>& pointCloud() const {
    return point_cloud_->pointCloud();
  }

  const PointT& operator[](const size_t index) const {
    return (*point_cloud_)[index];
  }

  virtual void reset() = 0;

  virtual size_t nearestKSearch(
      const PointT& point, size_t num_closest, std::vector<size_t>& k_indices,
      std::vector<FloatType>& k_squared_distances) const = 0;

  virtual size_t radiusSearch(const PointT& point, FloatType radius,
                              std::vector<size_t>& k_indices) const = 0;

  virtual size_t squaredRadiusSearch(const PointT& point,
                                     FloatType squared_radius,
                                     std::vector<size_t>& k_indices) const = 0;
};
}  // namespace kd_tree