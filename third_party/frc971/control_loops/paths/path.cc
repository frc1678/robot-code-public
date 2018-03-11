#include "third_party/frc971/control_loops/paths/path.h"
#include <Eigen/Geometry>

#define _USE_MATH_DEFINES
#include <cmath>

namespace frc971 {
namespace control_loops {
namespace paths {

Eigen::Vector2d Projection(Eigen::Vector2d a, Eigen::Vector2d direction) {
  return a.dot(direction) * direction.dot(direction) * direction;
}

Position FromMagDirection(double magnitude, double direction) {
  return magnitude * (Position() << ::std::cos(direction), ::std::sin(direction)).finished();
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
  new_values.block<2, 1>(0, 0) = Eigen::Rotation2D<double>(theta) * new_values.block<2, 1>(0, 0);

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

Pose Pose::Compose(const Pose &other) const { return other.RotateBy(heading()).TranslateBy(translational()); }

// TODO(Lyra): user specified velocity reference vector
// TODO(Lyra): angular velocity at endpoints
HermitePath::HermitePath(Pose initial, Pose final, double initial_velocity,
                         double final_velocity, bool backwards)
    : HermitePath(initial.translational(),
                  FromMagDirection(1, initial.heading()),
                  final.translational(),
                  FromMagDirection(1, final.heading()),
                  initial_velocity, final_velocity, backwards) {}

HermitePath::HermitePath(Position initial_position, Eigen::Vector2d initial_tangent,
                         Position final_position, Eigen::Vector2d final_tangent,
                         double initial_velocity, double final_velocity, bool backwards) {
  backwards_ = backwards;

  Eigen::Vector2d initial_derivative_basis;
  Eigen::Vector2d final_derivative_basis;

  {
    Eigen::Vector2d distance = final_position - initial_position;
    // How far to the side are we driving, relative to initial state?
    Eigen::Vector2d sideways = distance - Projection(distance, initial_tangent);
    // How far to the front is the sideways vector, relative to final state?
    double forwards2 = sideways.dot(final_tangent);
    if (backwards) {
      forwards2 *= -1;
    }
    // Rough estimate of the curvature, this should be approximately
    // correct as long as the curvature doesn't have excessively large changes.
    double approx_curve = (sideways.norm() * 2 - forwards2) / distance.norm();
    // Standard hermite spline uses tangent * |distance|, but initial velocity
    // should be taken into account as well, although only sigmificantly if
    // distance is short and initial velocity is high. This formula was found
    // experimentally.
    initial_derivative_basis = initial_tangent * (distance.norm() +
        initial_velocity * initial_velocity * 0.5 * approx_curve * approx_curve);
    final_derivative_basis = final_tangent * (distance.norm() +
        final_velocity * final_velocity * 0.5 * approx_curve * approx_curve);
  }

  if (backwards_) {
    initial_derivative_basis *= -1;
    final_derivative_basis *= -1;
  }

  coefficients_ = Eigen::Matrix<double, 4, 6>::Zero();
  Eigen::Vector2d initial_acceleration = Eigen::Vector2d::Zero();
  Eigen::Vector2d final_acceleration = Eigen::Vector2d::Zero();

  coefficients_.block<2, 1>(0, 0) = initial_position;
  coefficients_.block<2, 1>(0, 1) = initial_derivative_basis;
  coefficients_.block<2, 1>(0, 2) = 0.5 * initial_acceleration;
  coefficients_.block<2, 1>(0, 3) =
          -10 * initial_position - 6 * initial_derivative_basis +
          -1.5 * initial_acceleration + 0.5 * final_acceleration +
          -4 * final_derivative_basis + 10 * final_position;
  coefficients_.block<2, 1>(0, 4) =
          15 * initial_position + 8 * initial_derivative_basis +
          1.5 * initial_acceleration - 1 * final_acceleration +
          7 * final_derivative_basis - 15 * final_position;
  coefficients_.block<2, 1>(0, 5) =
          -6 * initial_position - 3 * initial_derivative_basis +
          -0.5 * initial_acceleration + 0.5 * final_acceleration +
          -3 * final_derivative_basis + 6 * final_position;

  for (int i = 0; i < 6; i++) {
    coefficients_.block<2, 1>(2, i) = coefficients_.block<2, 1>(0, i) * i;
  }

  initial_heading_ = remainder(::std::atan2(initial_tangent(1), initial_tangent(0)), 2 * M_PI);
}

void HermitePath::Populate(double s_min, double s_max, Pose *pose_arr, size_t arr_len) const {
  Eigen::Matrix<double, 6, 1> s_polynomial_bases;
  double step = (s_max - s_min) / (arr_len - 1);
  for (size_t i = 0; i < arr_len; i++) {
    double s = s_min + i * step;
    s_polynomial_bases << 1.0, s, s * s, s * s * s, s * s * s * s, s * s * s * s * s;

    Eigen::Vector4d combined = coefficients_ * s_polynomial_bases;

    double theta;
    if (s == 0) {
      // When s is _exactly_ zero, we can't get the heading directly from
      // the derivative (because it collapses to zero)! Let's use the cached
      // value instead.
      theta = initial_heading_;
    } else {
      theta = ::std::atan2(combined(3), combined(2));
      if (backwards_) {
        if (theta > 0) {
          theta -= M_PI;
        } else {
          theta += M_PI;
        }
      }
    }

    pose_arr[i] = Pose(combined.block<2, 1>(0, 0), theta);
  }
}

}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
