#ifndef MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_H_
#define MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_H_

#include <cmath>
#include <type_traits>
#include "muan/control/motion_profile.h"

namespace muan {
namespace control {

struct MotionProfileConstraints {
  muan::units::Velocity max_velocity;
  muan::units::Acceleration max_acceleration;
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
class TrapezoidalMotionProfile : public MotionProfile {
 public:
  TrapezoidalMotionProfile(MotionProfileConstraints constraints,
                           MotionProfilePosition goal)
      : TrapezoidalMotionProfile{constraints, goal,
                                 MotionProfilePosition{0, 0}} {}

  TrapezoidalMotionProfile(MotionProfileConstraints constraints,
                           MotionProfilePosition goal,
                           MotionProfilePosition initial);

  ~TrapezoidalMotionProfile() override = default;
  // Calculate the correct position and velocity for the profile at a time t
  // where the beginning of the profile was at time t=0
  MotionProfilePosition Calculate(muan::units::Time t) const override;

  muan::units::Time total_time() const override { return end_deccel_; }

  MotionProfileConstraints& constraints() { return constraints_; }

 private:
  // Is the profile inverted? In other words, does it need to increase or
  // decrease the velocity to get to the peak from the initial velocity?
  bool ShouldFlipAcceleration(
      const MotionProfilePosition& initial, const MotionProfilePosition& goal,
      const MotionProfileConstraints& constraints) const {
    // Calculate the distance travelled by a linear velocity ramp
    // from the initial to the final velocity and compare it to the desired
    // distance. If it is smaller, invert the profile.
    muan::units::Velocity velocity_change = goal.velocity - initial.velocity;

    muan::units::Length distance_change = goal.position - initial.position;

    muan::units::Time t =
        std::abs(velocity_change) / constraints.max_acceleration;
    bool is_acceleration_flipped =
        t * (velocity_change / 2 + initial.velocity) > distance_change;
    return is_acceleration_flipped;
  }

  // Flip the sign of the velocity and position if the profile is inverted
  MotionProfilePosition Direct(const MotionProfilePosition& in) const {
    MotionProfilePosition result = in;
    result.position *= direction_;
    result.velocity *= direction_;
    return result;
  }

  // The direction of the profile, either 1 for forwards or -1 for inverted
  int direction_;

  MotionProfileConstraints constraints_;
  MotionProfilePosition initial_, goal_;

  muan::units::Time end_accel_, end_full_speed_, end_deccel_;
};

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_H_
