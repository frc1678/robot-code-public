#include "muan/subsystems/drivetrain/open_loop_drive.h"
#include "muan/utils/math_utils.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

double HandleDeadband(double val, double deadband) {
  if (std::abs(val) < deadband) {
    val = 0;
  }
  return val;
}

void OpenLoopDrive::Update(OutputProto* output) {
  wheel_ = HandleDeadband(wheel_, kWheelDeadband);
  throttle_ = HandleDeadband(throttle_, kThrottleDeadband);

  double neg_inertia = wheel_ - old_wheel_;
  old_wheel_ = wheel_;

  double wheel_non_linearity;
  if (high_gear_) {
    wheel_non_linearity = dt_config_.high_gear_wheel_non_linearity;
    const double denominator = std::sin(M_PI / 2. * wheel_non_linearity);
    wheel_ = std::sin(M_PI / 2. * wheel_non_linearity * wheel_) / denominator;
    wheel_ = std::sin(M_PI / 2. * wheel_non_linearity * wheel_) / denominator;
  } else {
    wheel_non_linearity = dt_config_.low_gear_wheel_non_linearity;
    const double denominator = std::sin(M_PI / 2. * wheel_non_linearity);
    wheel_ = std::sin(M_PI / 2. * wheel_non_linearity * wheel_) / denominator;
    wheel_ = std::sin(M_PI / 2. * wheel_non_linearity * wheel_) / denominator;
    wheel_ = std::sin(M_PI / 2. * wheel_non_linearity * wheel_) / denominator;
  }

  double left, right, over_power;
  double sensitivity;

  double angular_power;
  double linear_power;

  double neg_inertia_scalar;
  if (high_gear_) {
    neg_inertia_scalar = kHighNegInertiaScalar;
    sensitivity = dt_config_.high_gear_sensitivity;
  } else {
    if (wheel_ * neg_inertia > 0) {
      neg_inertia_scalar = kLowNegInertiaTurnScalar;
    } else {
      if (std::abs(wheel_) > kLowNegInertiaThreshold) {
        neg_inertia_scalar = kLowNegInertiaFarScalar;
      } else {
        neg_inertia_scalar = kLowNegInertiaCloseScalar;
      }
    }
    sensitivity = dt_config_.low_gear_sensitivity;
  }

  double neg_inertia_power = neg_inertia * neg_inertia_scalar;
  neg_inertia_accum_ += neg_inertia_power;

  wheel_ = wheel_ + neg_inertia_accum_;
  if (neg_inertia_accum_ > 1) {
    neg_inertia_accum_ -= 1;
  } else if (neg_inertia_accum_ < -1) {
    neg_inertia_accum_ += 1;
  } else {
    neg_inertia_accum_ = 0;
  }

  linear_power = throttle_;

  if (quickturn_) {
    if (std::abs(linear_power) < kQuickStopDeadband) {
      double alpha = kQuickStopWeight;
      quick_stop_accum_ =
          (1 - alpha) * quick_stop_accum_ +
          alpha * muan::utils::Cap(wheel_, -1., 1.) * kQuickStopScalar;
    }
    over_power = 1.;
    angular_power = wheel_;
  } else {
    over_power = 0.;
    angular_power =
        std::abs(throttle_) * wheel_ * sensitivity - quick_stop_accum_;
    if (quick_stop_accum_ > 1) {
      quick_stop_accum_ -= 1;
    } else if (quick_stop_accum_ < -1) {
      quick_stop_accum_ += 1;
    } else {
      quick_stop_accum_ = 0;
    }
  }

  right = left = linear_power;
  left += angular_power;
  right -= angular_power;

  if (left > 1.) {
    right -= over_power * (left - 1.);
    left = 1.;
  } else if (right > 1.) {
    left -= over_power * (right - 1.);
    right = 1.;
  } else if (left < -1.) {
    right += over_power * (-left - 1.);
    left = -1.;
  } else if (right < -1.) {
    left += over_power * (-right - 1.);
    right = -1.;
  }

  (*output)->set_output_type(OPEN_LOOP);
  (*output)->set_left_setpoint(left);
  (*output)->set_right_setpoint(right);
}

void OpenLoopDrive::SetGoal(const GoalProto& goal) {
  auto teleop_goal = goal->teleop_goal();  // Auto because protos are a mess

  throttle_ = teleop_goal.throttle();
  wheel_ = teleop_goal.steering();
  quickturn_ = teleop_goal.quick_turn();

  high_gear_ = goal->high_gear();
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
