#include "third_party/aos/common/util/trapezoid_profile.h"
#include <iostream>

#include "third_party/aos/common/check.h"

using ::Eigen::Matrix;

namespace aos {
namespace util {

TrapezoidProfile::TrapezoidProfile(::std::chrono::nanoseconds delta_time)
    : maximum_acceleration_(0), maximum_velocity_(0), timestep_(delta_time) {
  output_.setZero();
}

void TrapezoidProfile::UpdateVals(double acceleration, double delta_time) {
  output_(0) +=
      output_(1) * delta_time + 0.5 * acceleration * delta_time * delta_time;
  output_(1) += acceleration * delta_time;
}

const Matrix<double, 2, 1> &TrapezoidProfile::Update(double goal_position,
                                                     double goal_velocity) {
  CalculateTimes(goal_position - output_(0), goal_velocity);

  double next_timestep =
      ::std::chrono::duration_cast<::std::chrono::duration<double>>(timestep_)
          .count();

  if (acceleration_time_ > next_timestep) {
    UpdateVals(acceleration_, next_timestep);
  } else {
    UpdateVals(acceleration_, acceleration_time_);

    next_timestep -= acceleration_time_;
    if (constant_time_ > next_timestep) {
      UpdateVals(0, next_timestep);
    } else {
      UpdateVals(0, constant_time_);
      next_timestep -= constant_time_;
      if (deceleration_time_ > next_timestep) {
        UpdateVals(deceleration_, next_timestep);
      } else {
        UpdateVals(deceleration_, deceleration_time_);
        next_timestep -= deceleration_time_;
        UpdateVals(0, next_timestep);
      }
    }
  }

  return output_;
}

void TrapezoidProfile::CalculateTimes(double distance_to_target,
                                      double goal_velocity) {
  if (distance_to_target == 0) {
    // We're there. Stop everything.
    // TODO(aschuh): Deal with velocity not right.
    acceleration_time_ = 0;
    acceleration_ = 0;
    constant_time_ = 0;
    deceleration_time_ = 0;
    deceleration_ = 0;
    return;
  } else if (distance_to_target < 0) {
    // Recurse with everything inverted.
    output_(1) *= -1;
    CalculateTimes(-distance_to_target, -goal_velocity);
    output_(1) *= -1;
    acceleration_ *= -1;
    deceleration_ *= -1;
    return;
  }

  constant_time_ = 0;
  acceleration_ = maximum_acceleration_;
  double maximum_acceleration_velocity =
      distance_to_target * 2 * std::abs(acceleration_) +
      output_(1) * output_(1);
  if (maximum_acceleration_velocity > 0) {
    maximum_acceleration_velocity = sqrt(maximum_acceleration_velocity);
  } else {
    maximum_acceleration_velocity = -sqrt(-maximum_acceleration_velocity);
  }

  // Since we know what we'd have to do if we kept after it to decelerate, we
  // know the sign of the acceleration.
  if (maximum_acceleration_velocity > goal_velocity) {
    deceleration_ = -maximum_acceleration_;
  } else {
    deceleration_ = maximum_acceleration_;
  }

  // We now know the top velocity we can get to.
  double top_velocity = sqrt(
      (distance_to_target + (output_(1) * output_(1)) / (2.0 * acceleration_) +
       (goal_velocity * goal_velocity) / (2.0 * deceleration_)) /
      (-1.0 / (2.0 * deceleration_) + 1.0 / (2.0 * acceleration_)));

  // If it can go too fast, we now know how long we get to accelerate for and
  // how long to go at constant velocity.
  if (top_velocity > maximum_velocity_) {
    acceleration_time_ =
        (maximum_velocity_ - output_(1)) / maximum_acceleration_;
    constant_time_ =
        (distance_to_target + (goal_velocity * goal_velocity -
                               maximum_velocity_ * maximum_velocity_) /
                                  (2.0 * maximum_acceleration_)) /
        maximum_velocity_;
  } else {
    acceleration_time_ = (top_velocity - output_(1)) / acceleration_;
  }

  CHECK_GT(top_velocity, -maximum_velocity_);

  if (output_(1) > maximum_velocity_) {
    constant_time_ = 0;
    acceleration_time_ = 0;
  }

  deceleration_time_ = (goal_velocity - top_velocity) / deceleration_;
}

double TrapezoidProfile::TimeLeftUntil(double x, double goal_position,
                                       double goal_velocity) const {
  double acceleration_distance = 0;
  double deceleration_distance = 0;
  double constant_distance = 0;
  double acceleration_time = std::max(acceleration_time_, 0.);
  double deceleration_time = std::max(deceleration_time_, 0.);
  double constant_time = std::max(constant_time_, 0.);

  if (std::abs(x - output_(0)) < 1e-6) {
    return 0;
  }
  
  if (std::abs(x - goal_position) < 1e-6) {
    return acceleration_time + constant_time + deceleration_time;
  }

  double distance_to_target = goal_position - output_(0);
  double top_velocity = sqrt(
      (distance_to_target + (output_(1) * output_(1)) / (2.0 * acceleration_) +
       (goal_velocity * goal_velocity) / (2.0 * deceleration_)) /
      (-1.0 / (2.0 * deceleration_) + 1.0 / (2.0 * acceleration_)));

  double distance_to_x = x - output_(0);

  if (distance_to_x < 0) {
    distance_to_x = 0;
  }

  top_velocity = std::min(top_velocity, maximum_velocity_);

  if (acceleration_time_ > 0) {
    acceleration_distance =
        output_(1) * acceleration_time_ +
        0.5 * acceleration_ * acceleration_time_ * acceleration_time_;
    constant_distance = top_velocity * constant_time_;
    deceleration_distance =
        top_velocity * deceleration_time_ +
        0.5 * deceleration_ * deceleration_time_ * deceleration_time_;
  } else {
    constant_distance = output_(1) * constant_time_;
    deceleration_distance =
        output_(1) * deceleration_time_ +
        0.5 * deceleration_ * deceleration_time_ * deceleration_time_;
  }

  if (acceleration_distance > distance_to_x) {
    acceleration_distance = distance_to_x;
    constant_distance = deceleration_distance = 0;
  } else if ((acceleration_distance + constant_distance > distance_to_x)) {
    constant_distance = distance_to_x - acceleration_distance;
    deceleration_distance = 0;
  } else {
    deceleration_distance =
        distance_to_x - constant_distance - acceleration_distance;
  }

  if (acceleration_distance > 0) {
    acceleration_time =
        (-output_(1) + sqrt(output_(1) * output_(1) +
                            2 * acceleration_ * acceleration_distance)) /
        acceleration_;
    constant_time = constant_distance / top_velocity;
    deceleration_time =
        (-top_velocity + sqrt(top_velocity * top_velocity +
                              2 * deceleration_ * deceleration_distance)) /
        deceleration_;
  } else {
    constant_time = constant_distance / output_(1);
    deceleration_time =
        (-output_(1) + sqrt(output_(1) * output_(1) +
                            2 * deceleration_ * deceleration_distance)) /
        deceleration_;
  }

  acceleration_time = std::max(0., acceleration_time);
  deceleration_time = std::max(0., deceleration_time);
  constant_time = std::max(0., constant_time);

  return acceleration_time + constant_time + deceleration_time;
}

}  // namespace util
}  // namespace aos
