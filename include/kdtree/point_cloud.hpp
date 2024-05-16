#pragma once

#include <memory>
#include <vector>

namespace point_cloud {
template <typename T>
class PointCloud {
 public:
  using PointT = T;
  using FloatType = typename PointT::FloatType;
  PointCloud() { point_cloud_.reset(new std::vector<PointT>); }
  PointCloud(const PointCloud& rhs) = default;
  PointCloud& operator=(const PointCloud& rhs) = default;
  virtual ~PointCloud() = default;

 protected:
  std::unique_ptr<std::vector<PointT>> point_cloud_;

 public:
  const std::vector<PointT>& pointCloud() const { return *point_cloud_; }

  const PointT& operator[](const size_t index) const {
    return (*point_cloud_)[index];
  }

  const size_t size() const { return kdtree_get_point_count(); }

  virtual void set(const std::vector<PointT>& point_cloud) {
    clear();
    add(point_cloud);
  }

  virtual void add(const std::vector<PointT>& point_cloud) {
    point_cloud_->insert(point_cloud_->end(), point_cloud.begin(),
                         point_cloud.end());
  }

  virtual void add(const PointT& point) { point_cloud_->push_back(point); }

  virtual void clear() { point_cloud_->clear(); }

  size_t kdtree_get_point_count() const { return point_cloud_->size(); }

  FloatType kdtree_get_pt(const size_t index, int dim) const {
    const PointT& point = (*point_cloud_)[index];
    if (dim == 0) {
      return point.x;
    } else if (dim == 1) {
      return point.y;
    } else if (dim == 2) {
      return point.z;
    }
    return 0.f;
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};
}  // namespace point_cloud