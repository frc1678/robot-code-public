#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"

#include "Eigen/Dense"
#include <cmath>
#include <memory>
#include <sched.h>
#include <stdio.h>

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/polydrivetrain.h"
#include "third_party/frc971/control_loops/drivetrain/ssdrivetrain.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

DrivetrainLoop::DrivetrainLoop(
    const DrivetrainConfig& dt_config,
    ::frc971::control_loops::drivetrain::GoalQueue* goal_queue,
    ::frc971::control_loops::drivetrain::InputQueue* input_queue,
    ::frc971::control_loops::drivetrain::OutputQueue* output_queue,
    ::frc971::control_loops::drivetrain::StatusQueue* status_queue,
    ::muan::wpilib::DriverStationQueue* driver_station_queue,
    ::muan::wpilib::gyro::GyroQueue* gyro_queue)
    : goal_queue_(goal_queue->MakeReader()),
      input_queue_(input_queue->MakeReader()),
      output_queue_(output_queue),
      status_queue_(status_queue),
      driver_station_queue_(driver_station_queue->MakeReader()),
      gyro_queue_(gyro_queue->MakeReader()),
      dt_config_(dt_config),
      kf_(dt_config_.make_kf_drivetrain_loop()),
      dt_openloop_(dt_config_, &kf_),
      cartesian_position_(Eigen::Matrix<double, 2, 1>::Zero()),
      dt_closedloop_(dt_config_, &kf_, &integrated_kf_heading_, &cartesian_position_),
      left_gear_(dt_config_.default_high_gear ? Gear::kHighGear
                                              : Gear::kLowGear),
      right_gear_(dt_config_.default_high_gear ? Gear::kHighGear
                                               : Gear::kLowGear),
      left_high_requested_(dt_config_.default_high_gear),
      right_high_requested_(dt_config_.default_high_gear) {
  ::aos::controls::HPolytope<0>::Init();
}

int DrivetrainLoop::ControllerIndexFromGears() {
  // 3 is high gear, 0 is low
  return left_gear_ ? 3 : 0;
}

void DrivetrainLoop::Update() {
  // Pull in the relevant queues and call RunIteration()
  auto input = input_queue_.ReadMessage();
  auto goal = goal_queue_.ReadLastMessage();

  ::frc971::control_loops::drivetrain::GoalProto* goal_ptr = nullptr;
  if (goal) {
    goal_ptr = &(goal.value());
  }

  ::frc971::control_loops::drivetrain::InputProto* input_ptr = nullptr;
  if (input) {
    input_ptr = &(input.value());
  }

  ::frc971::control_loops::drivetrain::OutputProto output;
  ::frc971::control_loops::drivetrain::StatusProto status;

  bool enable_outputs = true;

  auto driver_station = driver_station_queue_.ReadLastMessage();
  if (driver_station) {
    enable_outputs = (*driver_station)->is_sys_active();
  }

  if (enable_outputs) {
    RunIteration(goal_ptr, input_ptr, &output, &status);
  } else {
    RunIteration(goal_ptr, input_ptr, nullptr, &status);
    output->set_left_voltage(0.0);
    output->set_right_voltage(0.0);
    output->set_high_gear(dt_config_.default_high_gear);
  }

  output_queue_->WriteMessage(output);
  status_queue_->WriteMessage(status);
}

void DrivetrainLoop::RunIteration(
    const ::frc971::control_loops::drivetrain::GoalProto* goal,
    const ::frc971::control_loops::drivetrain::InputProto* input,
    ::frc971::control_loops::drivetrain::OutputProto* output,
    ::frc971::control_loops::drivetrain::StatusProto* status) {
  if (!has_been_enabled_ && output) {
    has_been_enabled_ = true;
  }

  // TODO(austin): Put gear detection logic here.
  switch (dt_config_.shifter_type) {
    case ShifterType::HALL_EFFECT_SHIFTER:
    case ShifterType::SIMPLE_SHIFTER:
      // Force the right controller for simple shifters since we assume that
      // gear switching is instantaneous.
      if (left_high_requested_) {
        left_gear_ = Gear::kHighGear;
      } else {
        left_gear_ = Gear::kLowGear;
      }
      if (right_high_requested_) {
        right_gear_ = Gear::kHighGear;
      } else {
        right_gear_ = Gear::kLowGear;
      }
      break;
    case ShifterType::NO_SHIFTER:
      break;
  }

  kf_.set_index(ControllerIndexFromGears());
  /*
  {
    GearLogging gear_logging;
    gear_logging.left_state = static_cast<uint32_t>(left_gear_);
    gear_logging.right_state = static_cast<uint32_t>(right_gear_);
    gear_logging.left_loop_high = MaybeHigh(left_gear_);
    gear_logging.right_loop_high = MaybeHigh(right_gear_);
    gear_logging.controller_index = kf_.controller_index();
    LOG_STRUCT(DEBUG, "state", gear_logging);
  }
  */

  // TODO(austin): Signal the current gear to both loops.

  auto gyro_message = gyro_queue_.ReadLastMessage();
  if (gyro_message) {
    // LOG_STRUCT(DEBUG, "using", *gyro_reading.get());
    last_gyro_heading_ = (*gyro_message)->current_angle();
    last_gyro_rate_ = (*gyro_message)->current_angular_velocity();
  }

  {
    Eigen::Matrix<double, 3, 1> Y;
    Y << (*input)->left_encoder(), (*input)->right_encoder(), last_gyro_rate_;

    kf_.Correct(Y);
    integrated_kf_heading_ += dt_config_.dt *
                              (kf_.X_hat(3, 0) - kf_.X_hat(1, 0)) /
                              (dt_config_.robot_radius * 2.0);

    // gyro_heading = (real_right - real_left) / width
    // wheel_heading = (wheel_right - wheel_left) / width
    // gyro_heading + offset = wheel_heading
    // gyro_goal + offset = wheel_goal
    // offset = wheel_heading - gyro_heading

    // gyro_goal + wheel_heading - gyro_heading = wheel_goal
  }

  dt_openloop_.SetPosition(input, left_gear_, right_gear_);

  bool control_loop_driving = false;
  if (goal) {
    // TODO(Kyle) Check this condition
    control_loop_driving = (*goal)->has_distance_command() || (*goal)->has_path_command();

    dt_closedloop_.SetGoal(*goal);
    dt_openloop_.SetGoal(*goal);
  }

  dt_openloop_.Update();

  if (control_loop_driving) {
    dt_closedloop_.Update(output != NULL);
    dt_closedloop_.SetOutput(output);
  } else {
    dt_openloop_.SetOutput(output);
    // TODO(austin): Set profile to current spot.
    dt_closedloop_.Update(false);
  }

  {
    Eigen::Matrix<double, 2, 1> linear =
        dt_closedloop_.LeftRightToLinear(kf_.X_hat());
    cartesian_position_(0) += linear(1) * ::std::cos(integrated_kf_heading_) * 0.005;
    cartesian_position_(1) += linear(1) * ::std::sin(integrated_kf_heading_) * 0.005;
  }

  // The output should now contain the shift request.

  // set the output status of the control loop state
  if (status) {
    (*status)->set_forward_velocity((kf_.X_hat(1, 0) + kf_.X_hat(3, 0)) / 2.0);

    Eigen::Matrix<double, 2, 1> linear =
        dt_closedloop_.LeftRightToLinear(kf_.X_hat());
    Eigen::Matrix<double, 2, 1> angular =
        dt_closedloop_.LeftRightToAngular(kf_.X_hat());

    angular(0, 0) = integrated_kf_heading_;

    Eigen::Matrix<double, 4, 1> gyro_left_right =
        dt_closedloop_.AngularLinearToLeftRight(linear, angular);

    (*status)->set_estimated_left_position(gyro_left_right(0, 0));
    (*status)->set_estimated_right_position(gyro_left_right(2, 0));

    (*status)->set_estimated_left_velocity(gyro_left_right(1, 0));
    (*status)->set_estimated_right_velocity(gyro_left_right(3, 0));
    (*status)->set_output_was_capped(dt_closedloop_.output_was_capped());
    (*status)->set_uncapped_left_voltage(kf_.U_uncapped(0, 0));
    (*status)->set_uncapped_right_voltage(kf_.U_uncapped(1, 0));

    (*status)->set_left_voltage_error(kf_.X_hat(4, 0));
    (*status)->set_right_voltage_error(kf_.X_hat(5, 0));
    (*status)->set_estimated_angular_velocity_error(kf_.X_hat(6, 0));
    (*status)->set_estimated_heading(integrated_kf_heading_);

    (*status)->set_estimated_x_position(cartesian_position_(0));
    (*status)->set_estimated_y_position(cartesian_position_(1));

    dt_openloop_.PopulateStatus(status);
    dt_closedloop_.PopulateStatus(status);
  }

  double left_voltage = 0.0;
  double right_voltage = 0.0;
  if (output) {
    left_voltage = (*output)->left_voltage();
    right_voltage = (*output)->right_voltage();
    left_high_requested_ = right_high_requested_ = (*output)->high_gear();
  }

  const double scalar = 1.0;  // ::aos::robot_state->voltage_battery / 12.0;

  left_voltage *= scalar;
  right_voltage *= scalar;

  // To validate, look at the following:

  // Observed - dx/dt velocity for left, right.

  // Angular velocity error compared to the gyro
  // Gyro heading vs left-right
  // Voltage error.

  Eigen::Matrix<double, 2, 1> U;
  U << last_left_voltage_, last_right_voltage_;
  last_left_voltage_ = left_voltage;
  last_right_voltage_ = right_voltage;

  kf_.UpdateObserver(U, ::std::chrono::milliseconds(5));
}

void DrivetrainLoop::Zero(
    ::frc971::control_loops::drivetrain::OutputProto* output) {
  (*output)->set_left_voltage(0);
  (*output)->set_right_voltage(0);
  (*output)->set_high_gear(dt_config_.default_high_gear);
}

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
