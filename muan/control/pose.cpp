#include "muan/control/pose.h"
#include <Eigen/Geometry>

namespace muan {
namespace control {

Eigen::Vector2d Projection(Eigen::Vector2d a, Eigen::Vector2d direction) {
  return a.dot(direction) * direction.dot(direction) * direction;
}

Eigen::Vector2d FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Eigen::Vector2d() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

Pose::Pose(Eigen::Vector3d values) : values_(values) {}

Pose::Pose(Eigen::Vector2d pos, double theta) {
  values_.block<2, 1>(0, 0) = pos;
  values_(2) = remainder(theta, 2 * M_PI);
}

Pose Pose::operator+(const Pose &other) const {
  Eigen::Vector3d new_values = values_ + other.values_;

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2), 2 * M_PI);

  return Pose(new_values);
}

Pose Pose::TranslateBy(const Eigen::Vector2d &delta) const {
  Eigen::Vector3d new_values = values_;
  new_values.block<2, 1>(0, 0) += delta;
  return Pose(new_values);
}

Pose Pose::RotateBy(double theta) const {
  Eigen::Vector3d new_values = values_;
  new_values.block<2, 1>(0, 0) =
      Eigen::Rotation2D<double>(theta) * new_values.block<2, 1>(0, 0);

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2) + theta, 2 * M_PI);

  return Pose(new_values);
}

Pose Pose::operator-(const Pose &other) const {
  Eigen::Vector3d new_values = values_ - other.values_;

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2), 2 * M_PI);
  return Pose(new_values);
}

Pose Pose::Compose(const Pose &other) const {
  return other.RotateBy(heading()).TranslateBy(translational());
}

Pose Pose::Interpolate(Pose other, double frac) const {
  if (frac <= 0) {
    return Pose(Get());
  } else if (frac >= 1) {
    return Pose(other.Get());
  }
  Pose delta = Pose((other.translational() - translational()) * frac,
                    remainder((other.heading() - heading()) * frac, 2 * M_PI));
  Pose result = Compose(delta);

  return result;
}

PoseWithCurvature::PoseWithCurvature(Pose pose, double curvature)
    : pose_(pose), curvature_(curvature) {}

PoseWithCurvature PoseWithCurvature::operator+(
    const PoseWithCurvature &other) const {
  return PoseWithCurvature(pose_ + other.pose_, curvature_);
}

PoseWithCurvature PoseWithCurvature::operator-(
    const PoseWithCurvature &other) const {
  return PoseWithCurvature(pose_ - other.pose_, curvature_);
}

PoseWithCurvature PoseWithCurvature::TranslateBy(
    const Eigen::Vector2d &delta) const {
  Eigen::Vector2d new_values = pose_.translational() + delta;
  return PoseWithCurvature(Pose(new_values, pose_.heading()), curvature_);
}

PoseWithCurvature PoseWithCurvature::Interpolate(PoseWithCurvature other,
                                                 double frac) const {
  return PoseWithCurvature(
      pose_.Interpolate(other.pose(), frac),
      curvature_ + ((other.curvature() - curvature_) * frac));
}

}  // namespace control
}  // namespace muan
