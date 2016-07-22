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
  MotionProfileConstraints<DistanceType> constraints_;
  MotionProfilePosition<DistanceType> initial_, goal_;

  Time end_accel_, end_full_speed_, end_deccel_;
};

} /* control */

} /* muan */

#include "trapezoidal_motion_profile.hpp"

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
