#include "third_party/frc971/control_loops/paths/path.h"
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>

namespace frc971 {
namespace control_loops {
namespace paths {

Position FromMagDirection(double magnitude, double direction) {
  return magnitude *
         (Position() << ::std::cos(direction), ::std::sin(direction))
             .finished();
}

Pose::Pose(Eigen::Vector3d values) : values_(values) {}

Pose::Pose(Position pos, double theta) {
  values_.block<2, 1>(0, 0) = pos;
  values_(2) = remainder(theta, 2 * M_PI);
}

Pose Pose::operator+(const Pose &other) const {
  Eigen::Vector3d new_values = values_ + other.values_;

  // Wrap the heading into [-pi, pi]
  new_values(2) = remainder(new_values(2), 2 * M_PI);

  return Pose(new_values);
}

Pose Pose::TranslateBy(const Position &delta) const {
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

HermitePath::HermitePath(Pose initial, Pose final)
    : HermitePath(initial.translational(),
                  FromMagDirection((final - initial).translational().norm(),
                                   initial.heading()),
                  final.translational(),
                  FromMagDirection((final - initial).translational().norm(),
                                   final.heading())) {}

HermitePath::HermitePath(Position initial_position,
                         Eigen::Vector2d initial_tangent,
                         Position final_position,
                         Eigen::Vector2d final_tangent) {
  coefficients_ = Eigen::Matrix<double, 4, 4>::Zero();
  coefficients_.block<2, 1>(0, 0) = initial_position;
  coefficients_.block<2, 1>(0, 1) = initial_tangent;
  coefficients_.block<2, 1>(0, 2) = 3 * (final_position - initial_position) -
                                    2 * initial_tangent - final_tangent;
  coefficients_.block<2, 1>(0, 3) =
      initial_tangent + final_tangent + 2 * (initial_position - final_position);

  for (int i = 0; i < 4; i++) {
    coefficients_.block<2, 1>(2, i) = coefficients_.block<2, 1>(0, i) * i;
  }

  initial_heading_ =
      remainder(::std::atan2(initial_tangent(1), initial_tangent(0)), 2 * M_PI);
}

void HermitePath::Populate(double s_min, double s_max, Pose *pose_arr,
                           size_t arr_len) const {
  Eigen::Matrix<double, 4, 1> s_polynomial_bases;
  double step = (s_max - s_min) / (arr_len - 1);
  for (size_t i = 0; i < arr_len; i++) {
    double s = s_min + i * step;
    s_polynomial_bases << 1.0, s, s * s, s * s * s;

    Eigen::Vector4d combined = coefficients_ * s_polynomial_bases;

    double theta = ::std::atan2(combined(3), combined(2));
    if (s == 0) {
      // When s is _exactly_ zero, we can't get the heading directly from
      // the derivative (because it collapses to zero)! Let's use the cached
      // value instead.
      theta = initial_heading_;
    }

    pose_arr[i] = Pose(combined.block<2, 1>(0, 0), theta);
  }
}

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
