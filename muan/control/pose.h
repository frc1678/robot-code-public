#ifndef MUAN_CONTROL_POSE_H_
#define MUAN_CONTROL_POSE_H_

#include "Eigen/Core"

namespace muan {
namespace control {

Eigen::Vector2d Projection(Eigen::Vector2d a, Eigen::Vector2d direction);
Eigen::Vector2d FromMagDirection(double magnitude, double direction);

class Pose {
 public:
  Pose() = default;
  Pose(Eigen::Vector2d position, double theta);
  explicit Pose(Eigen::Vector3d values);

  Pose operator+(const Pose &other) const;
  Pose operator-(const Pose &other) const;

  inline Eigen::Vector3d Get() const { return values_; }
  inline Eigen::Vector2d translational() const {
    return values_.block<2, 1>(0, 0);
  }
  inline double heading() const { return values_(2); }

  Pose TranslateBy(const Eigen::Vector2d &delta) const;
  Pose RotateBy(double theta) const;

  Pose Interpolate(Pose other, double frac) const;

  // Compose this pose with another. Treat the new pose as an offset, using this
  // pose as the origin (theta=x axis)
  Pose Compose(const Pose &other) const;

 private:
  Eigen::Vector3d values_;
};

class PoseWithCurvature {
 public:
  PoseWithCurvature() = default;
  PoseWithCurvature(Pose pose, double curvature);

  PoseWithCurvature operator+(const PoseWithCurvature &other) const;
  PoseWithCurvature operator-(const PoseWithCurvature &other) const;

  PoseWithCurvature TranslateBy(const Eigen::Vector2d &delta) const;

  PoseWithCurvature Interpolate(PoseWithCurvature other, double frac) const;

  inline Eigen::Vector2d translational() const { return pose_.translational(); }
  inline double heading() const { return pose_.heading(); }

  inline Eigen::Matrix<double, 4, 1> Get() const {
    return (Eigen::Matrix<double, 4, 1>() << pose_.Get(), curvature_)
        .finished();
  }

  inline Pose pose() const { return pose_; }
  inline double curvature() const { return curvature_; }

 private:
  Pose pose_;
  double curvature_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_POSE_H_
