#include "muan/control/spline.h"
#include "muan/utils/math_utils.h"

namespace muan {
namespace control {

HermiteSpline::HermiteSpline(Pose initial, Pose final, double initial_velocity,
                             double final_velocity, bool backwards,
                             double extra_distance_initial,
                             double extra_distance_final,
                             double initial_angular_velocity,
                             double final_angular_velocity)
    : HermiteSpline(initial.translational(),
                    FromMagDirection(1, initial.heading()),
                    final.translational(), FromMagDirection(1, final.heading()),
                    initial_velocity, final_velocity, backwards,
                    extra_distance_initial, extra_distance_final,
                    initial_angular_velocity, final_angular_velocity) {}

HermiteSpline::HermiteSpline(
    Position initial_position, Eigen::Vector2d initial_tangent,
    Position final_position, Eigen::Vector2d final_tangent,
    double initial_velocity, double final_velocity, bool backwards,
    double extra_distance_initial, double extra_distance_final,
    double initial_angular_velocity, double final_angular_velocity) {
  backwards_ = backwards;

  Eigen::Vector2d initial_derivative_basis;
  Eigen::Vector2d final_derivative_basis;
  double initial_deriv_magnitude;
  double final_deriv_magnitude;

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
    initial_deriv_magnitude =
        distance.norm() + extra_distance_initial * 5.0 +
        initial_velocity * initial_velocity * 0.5 * approx_curve * approx_curve;
    final_deriv_magnitude =
        distance.norm() + extra_distance_final * 5.0 +
        final_velocity * final_velocity * 0.5 * approx_curve * approx_curve;
  }

  initial_derivative_basis = initial_tangent * initial_deriv_magnitude;
  final_derivative_basis = final_tangent * final_deriv_magnitude;

  if (backwards_) {
    initial_derivative_basis *= -1;
    final_derivative_basis *= -1;
  }

  Eigen::Vector2d initial_acceleration = Eigen::Vector2d::Zero();
  if (initial_velocity > 0.01 || initial_velocity < -0.01) {
    initial_acceleration =
        (Eigen::Vector2d() << -initial_tangent(1), initial_tangent(0))
            .finished() *
        initial_deriv_magnitude * final_deriv_magnitude *
        initial_angular_velocity / initial_velocity;
  } else if (initial_angular_velocity > 0.01 ||
             initial_angular_velocity < -0.01) {
  }

  Eigen::Vector2d final_acceleration = Eigen::Vector2d::Zero();
  if (final_velocity > 0.01 || final_velocity < -0.01) {
    final_acceleration =
        (Eigen::Vector2d() << -final_tangent(1), final_tangent(0)).finished() *
        final_deriv_magnitude * final_deriv_magnitude * final_angular_velocity /
        final_velocity;
  } else if (final_angular_velocity > 0.01 || final_angular_velocity < -0.01) {
  }

  coefficients_ = Eigen::Matrix<double, 4, 6>::Zero();
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

  initial_heading_ =
      remainder(::std::atan2(initial_tangent(1), initial_tangent(0)), 2 * M_PI);
}

std::array<PoseWithCurvature, kNumSamples> HermiteSpline::Populate(
    double s_min, double s_max) const {
  Eigen::Matrix<double, 6, 1> s_polynomial_bases;
  double step = (s_max - s_min) / (kNumSamples - 1);
  std::array<PoseWithCurvature, kNumSamples> pose_arr;

  Eigen::Vector4d combined;

  for (int i = 0; i < kNumSamples; i++) {
    double s = s_min + i * step;

    s_polynomial_bases << 1.0, s, s * s, s * s * s, s * s * s * s,
        s * s * s * s * s;

    combined = coefficients_ * s_polynomial_bases;

    double heading;
    double curvature;
    if (s == 0) {
      // When s is _exactly_ zero, we can't get the heading directly from
      // the derivative (because it collapses to zero)! Let's use the cached
      // value instead.
      heading = initial_heading_;
      curvature = 0;
    } else {
      heading = ::std::atan2(combined(3), combined(2));
      if (backwards_) {
        if (heading > 0) {
          heading -= M_PI;
        } else {
          heading += M_PI;
        }
      }

      if (i > 0) {
        Eigen::Vector2d tangent = FromMagDirection(1., heading);
        Eigen::Vector2d prev_tangent =
            FromMagDirection(1., pose_arr.at(i - 1).heading());

        Eigen::Vector2d dT = tangent - prev_tangent;
        double ds =
            (combined.block<2, 1>(0, 0) - pose_arr.at(i - 1).translational())
                .norm();
        curvature =
            (dT / ds).norm() * (backwards_ ? -1. : 1.) *
            std::copysign(
                1., heading -
                        pose_arr.at(i - 1).heading());  // signed curvature (k)
      } else {
        curvature = 0.;
      }
    }

    pose_arr.at(i) =
        PoseWithCurvature(Pose(combined.block<2, 1>(0, 0), heading), curvature);
  }
  return pose_arr;
}

}  // namespace control
}  // namespace muan
