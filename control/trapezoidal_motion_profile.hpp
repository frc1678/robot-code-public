#ifndef MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_HPP_
#define MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_HPP_

#include "trapezoidal_motion_profile.h"
#include <iostream>

namespace muan {

namespace control {

template <typename DistanceType>
TrapezoidalMotionProfile<DistanceType>::TrapezoidalMotionProfile(
    MotionProfileConstraints<DistanceType> constraints,
    MotionProfilePosition<DistanceType> goal,
    MotionProfilePosition<DistanceType> initial)
    : constraints_{constraints},
      initial_{initial},
      goal_{goal} {
  Time cutoff_begin = initial_.velocity / constraints_.max_acceleration;
  DistanceType cutoff_dist_begin =
      cutoff_begin * cutoff_begin * constraints_.max_acceleration / 2.0;

  Time cutoff_end = goal_.velocity / constraints_.max_acceleration;
  DistanceType cutoff_dist_end =
      cutoff_end * cutoff_end * constraints_.max_acceleration / 2.0;

  // Now we can calculate the parameters as if it was a full trapezoid instead
  // of a truncated one
  {
    auto full_trapezoid_dist = cutoff_dist_begin +
                               (goal_.position - initial_.position) +
                               cutoff_dist_end;
    auto acceleration_time =
        constraints_.max_velocity / constraints_.max_acceleration;

    auto full_speed_dist =
        full_trapezoid_dist -
        acceleration_time * acceleration_time * constraints_.max_acceleration;

    if (full_speed_dist < DistanceType{0}) {
      acceleration_time =
          std::sqrt((full_trapezoid_dist / constraints_.max_acceleration)()) *
          s;
      full_speed_dist = DistanceType{0};
    }

    end_accel_ = acceleration_time - cutoff_begin;
    end_full_speed_ = end_accel_ + full_speed_dist / constraints_.max_velocity;
    end_deccel_ = end_full_speed_ + acceleration_time - cutoff_end;
  }
}

template <typename DistanceType>
MotionProfilePosition<DistanceType>
TrapezoidalMotionProfile<DistanceType>::Calculate(Time t) const {
  MotionProfilePosition<DistanceType> result = initial_;

  if (t < end_accel_) {
    result.velocity += t * constraints_.max_acceleration;
    result.position +=
        (initial_.velocity + t * constraints_.max_acceleration / 2.0) * t;
  } else if (t < end_full_speed_) {
    result.velocity = constraints_.max_velocity;
    result.position +=
        (initial_.velocity + end_accel_ * constraints_.max_acceleration / 2.0) *
            end_accel_ +
        constraints_.max_velocity * (t - end_accel_);
  } else if (t <= end_deccel_) {
    result.velocity =
        goal_.velocity + (end_deccel_ - t) * constraints_.max_acceleration;
    Time time_left = end_deccel_ - t;
    result.position =
        goal_.position -
        (goal_.velocity + time_left * constraints_.max_acceleration / 2.0) *
            time_left;
  } else {
    result = goal_;
  }

  return result;
}

} /* control */

} /* muan */

#endif /* MUAN_CONTROL_TRAPEZOIDAL_MOTION_PROFILE_HPP_ */
