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

/*
 * A trapezoidal-shaped velocity profile.
 * Example:
 *    MotionProfileConstraints constraints{ 1 * m / s, 1 * m / s / s };
 *    TrapezoidalMotionProfile<Length> profile{ constraints,
 *                                              current_position,
 *                                              goal_position };
 *    ...
 *    desired_position = profile.Calculate(t - profile_start_time);
 */
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

  ~TrapezoidalMotionProfile() override = default;

  // Calculate the correct position and velocity for the profile at a time t
  // where the beginning of the profile was at time t=0
  MotionProfilePosition<DistanceType> Calculate(Time t) const override;

  Time total_time() const override { return end_deccel_; }

  MotionProfileConstraints<DistanceType>& constraints() { return constraints_; }

 private:
  // Is the profile inverted? In other words, does it need to increase or
  // decrease the velocity to get to the peak from the initial velocity?
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

  // Flip the sign of the velocity and position if the profile is inverted
  MotionProfilePosition<DistanceType> Direct(
      const MotionProfilePosition<DistanceType>& in) const {
    MotionProfilePosition<DistanceType> result = in;
    result.position *= direction_;
    result.velocity *= direction_;
    return result;
  }

  // The direction of the profile, either 1 for forwards or -1 for inverted
  int direction_;

  MotionProfileConstraints<DistanceType> constraints_;
  MotionProfilePosition<DistanceType> initial_, goal_;

  Time end_accel_, end_full_speed_, end_deccel_;
};

}  // namespace control

}  // namespace muan

#include "trapezoidal_motion_profile.hpp"

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
