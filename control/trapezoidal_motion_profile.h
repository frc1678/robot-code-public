#ifndef MUAN_CONTROL_TRAPEZOIDALMOTIONPROFILE_H_
#define MUAN_CONTROL_TRAPEZOIDALMOTIONPROFILE_H_

#include "motion_profile.h"
#include <cmath>
#include <type_traits>

namespace muan {

namespace control {

template <typename DistanceType>
struct MotionProfileConstraints {
  TimeDerivative<DistanceType> max_velocity;
  TimeDerivative2<DistanceType> max_acceleration;
};

template <typename DistanceType>
class TrapezoidalMotionProfile : public MotionProfile<DistanceType> {
 public:
  TrapezoidalMotionProfile(MotionProfileConstraints<DistanceType> constraints,
                           MotionProfilePosition<DistanceType> goal)
      : TrapezoidalMotionProfile<DistanceType>{
            constraints, goal, MotionProfilePosition<DistanceType>{0, 0}} {}

  TrapezoidalMotionProfile(MotionProfileConstraints<DistanceType> constraints,
                           MotionProfilePosition<DistanceType> goal,
                           MotionProfilePosition<DistanceType> initial);

  virtual ~TrapezoidalMotionProfile() = default;

  MotionProfilePosition<DistanceType> Calculate(Time t) const override;

  Time total_time() const override { return end_deccel_; }

  MotionProfileConstraints<DistanceType>& constraints() { return constraints_; }

 private:
  bool ShouldFlipAcceleration(
      const MotionProfilePosition<DistanceType>& initial,
      const MotionProfilePosition<DistanceType>& goal,
      const MotionProfileConstraints<DistanceType>& constraints) const {
    // Calculate the distance travelled by a linear velocity ramp
    // from the initial to the final velocity and compare it to the desired
    // distance. If it is smaller, invert the profile.
    TimeDerivative<DistanceType> velocity_change =
        goal.velocity - initial.velocity;
    DistanceType distance_change = goal.position - initial.position;
    Time t = muan::abs(velocity_change) / constraints.max_acceleration;
    bool is_acceleration_flipped =
        t * (velocity_change / 2 + initial.velocity) > distance_change;
    return is_acceleration_flipped;
  }

  MotionProfilePosition<DistanceType> Direct(
      const MotionProfilePosition<DistanceType>& in) const {
    MotionProfilePosition<DistanceType> result = in;
    result.position *= direction_;
    result.velocity *= direction_;
    return result;
  }

  int direction_;

  MotionProfileConstraints<DistanceType> constraints_;
  MotionProfilePosition<DistanceType> initial_, goal_;

  Time end_accel_, end_full_speed_, end_deccel_;
};

} /* control */

} /* muan */

#include "trapezoidal_motion_profile.hpp"

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
