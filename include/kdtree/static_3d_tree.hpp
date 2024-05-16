#pragma once

#include "kd_tree.hpp"

namespace kd_tree {
template <typename PointCloudT>
class Static3dTree : public KdTree<PointCloudT> {
 public:
  using PointT = typename PointCloudT::PointT;
  using FloatType = typename PointT::FloatType;
  explicit Static3dTree() {
    kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }
  virtual ~Static3dTree() = default;

 private:
  using KdTree<PointCloudT>::point_cloud_;
  using KdTree<PointCloudT>::params_;
  using KdTreeBaseT = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::SO3_Adaptor<FloatType, PointCloudT>, PointCloudT, 3>;
  std::unique_ptr<KdTreeBaseT> kd_tree_base_;

 public:
  void add(const PointT& point) { point_cloud_->add(point); }

  void build() { kd_tree_base_->buildIndex(); }

  void build(const std::vector<PointT>& point_cloud) {
    point_cloud_->set(point_cloud);
    build();
  }

  void reset() override {
    point_cloud_->clear();
    kd_tree_base_.reset(new KdTreeBaseT(3, *point_cloud_));
  }

  size_t nearestKSearch(
      const PointT& point, size_t num_closest, std::vector<size_t>& k_indices,
      std::vector<FloatType>& k_squared_distances) const override {
    k_indices.resize(num_closest);
    k_squared_distances.resize(num_closest);
    nanoflann::KNNResultSet<FloatType> resultSet(num_closest);
    resultSet.init(k_indices.data(), k_squared_distances.data());
    kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    return resultSet.size();
  }

  size_t radiusSearch(const PointT& point, FloatType radius,
                      std::vector<size_t>& k_indices) const override {
    return squaredRadiusSearch(point, radius * radius, k_indices);
  }

  size_t squaredRadiusSearch(const PointT& point, FloatType squared_radius,
                             std::vector<size_t>& k_indices) const override {
    std::vector<std::pair<size_t, FloatType>> indices_dist;
    indices_dist.reserve(128);
    nanoflann::RadiusResultSet<FloatType> resultSet(squared_radius,
                                                    indices_dist);
    kd_tree_base_->findNeighbors(resultSet, point.data, params_);
    const size_t nFound = resultSet.size();
    if (params_.sorted) {
      std::sort(indices_dist.begin(), indices_dist.end(),
                nanoflann::IndexDist_Sorter());
    }
    k_indices.resize(nFound);
    for (size_t i = 0; i < nFound; i++) {
      k_indices[i] = indices_dist[i].first;
    }
    return nFound;
  }
};
}  // namespace kd_tree