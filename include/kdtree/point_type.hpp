#pragma once

#include <Eigen/Dense>
#include <iostream>

#define POINT3F                                                  \
  union EIGEN_ALIGN16 {                                          \
    float data[3];                                               \
    struct {                                                     \
      float x;                                                   \
      float y;                                                   \
      float z;                                                   \
    };                                                           \
  };                                                             \
  inline Eigen::Map<Eigen::Vector3f> point() {                   \
    return (Eigen::Vector3f::Map(data));                         \
  }                                                              \
  inline const Eigen::Map<const Eigen::Vector3f> point() const { \
    return (Eigen::Vector3f::Map(data));                         \
  }

#define POINT3D                                                  \
  union EIGEN_ALIGN16 {                                          \
    double data[3];                                              \
    struct {                                                     \
      double x;                                                  \
      double y;                                                  \
      double z;                                                  \
    };                                                           \
  };                                                             \
  inline Eigen::Map<Eigen::Vector3d> point() {                   \
    return (Eigen::Vector3d::Map(data));                         \
  }                                                              \
  inline const Eigen::Map<const Eigen::Vector3d> point() const { \
    return (Eigen::Vector3d::Map(data));                         \
  }

namespace point_type {
struct EIGEN_ALIGN16 Point3f {
  POINT3F;
  using FloatType = float;
  Point3f() = default;
  explicit Point3f(const float x_, const float y_, const float z_)
      : x(x_), y(y_), z(z_) {}
  explicit Point3f(const Eigen::Vector3f& point) { this->point() = point; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& os, const Point3f& p) {
  os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return os;
}

struct EIGEN_ALIGN16 Point3d {
  POINT3D;
  using FloatType = double;
  Point3d() = default;
  explicit Point3d(const double x_, const double y_, const double z_)
      : x(x_), y(y_), z(z_) {}
  explicit Point3d(const Eigen::Vector3d& point) { this->point() = point; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::ostream& operator<<(std::ostream& os, const Point3d& p) {
  os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
  return os;
}
}  // namespace point_type