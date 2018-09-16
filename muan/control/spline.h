#ifndef MUAN_CONTROL_SPLINE_H_
#define MUAN_CONTROL_SPLINE_H_

#include <vector>
#include "Eigen/Core"
#include "muan/control/pose.h"

namespace muan {
namespace control {

using Position = Eigen::Vector2d;

constexpr int kNumSamples = 1001;

class HermiteSpline {
 public:
  HermiteSpline(Pose initial, Pose final, double initial_velocity,
                double final_velocity, bool backwards,
                double extra_distance_initial, double extra_distance_final,
                double initial_angular_velocity, double final_angular_velocity);
  HermiteSpline(Position initial_position, Eigen::Vector2d initial_tangent,
                Position final_position, Eigen::Vector2d final_tangent,
                double initial_velocity, double final_velocity, bool backwards,
                double extra_distance_initial, double extra_distance_final,
                double initial_angular_velocity, double final_angular_velocity);

  std::array<PoseWithCurvature, kNumSamples> Populate(double s_min,
                                                      double s_max) const;

  inline bool backwards() const { return backwards_; }

 private:
  // (1, s, s^2, s^3, s^4, s^5) -> (x, y, x', y')
  Eigen::Matrix<double, 4, 6> coefficients_;

  // Cached, because it is lost in the first-derivative at s=0
  double initial_heading_;

  bool backwards_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_SPLINE_H_
