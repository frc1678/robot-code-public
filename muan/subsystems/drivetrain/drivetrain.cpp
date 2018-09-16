#include "muan/subsystems/drivetrain/drivetrain.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using muan::control::DrivetrainModel;
using muan::control::DriveTransmission;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

Drivetrain::Drivetrain(DrivetrainConfig dt_config)
    : drive_model_{dt_config.drive_properties,
                   DriveTransmission(dt_config.low_gear_properties),
                   DriveTransmission(dt_config.high_gear_properties)},
      input_reader_{QueueManager<InputProto>::Fetch()->MakeReader()},
      goal_reader_{QueueManager<GoalProto>::Fetch()->MakeReader()},
      ds_status_reader_{
          QueueManager<DriverStationProto>::Fetch()->MakeReader()},
      output_queue_{QueueManager<OutputProto>::Fetch()},
      status_queue_{QueueManager<StatusProto>::Fetch()},
      dt_config_{dt_config},
      open_loop_{dt_config},
      closed_loop_{dt_config, &cartesian_position_, &integrated_heading_,
                   &linear_angular_velocity_} {}

void Drivetrain::Update() {
  InputProto input;
  OutputProto output;
  StatusProto status;
  GoalProto goal;
  DriverStationProto driver_station;

  if (!input_reader_.ReadLastMessage(&input)) {
    // We _really_ need these
    // TODO(jishnu) find an elegant way to handle this
    return;
  }

  if (!ds_status_reader_.ReadLastMessage(&driver_station)) {
    // Even if we don't get a message, we know that it is a 12V battery
    driver_station->set_battery_voltage(12.0);
  }

  const double delta_left = input->left_encoder() - prev_left_;
  const double delta_right = input->right_encoder() - prev_right_;
  const double delta_heading = input->gyro() - prev_heading_;

  prev_left_ = input->left_encoder();
  prev_right_ = input->right_encoder();
  prev_heading_ = input->gyro();

  double delta_linear = drive_model_.ForwardKinematics(
      Eigen::Vector2d(delta_left, delta_right))(0);

  linear_angular_velocity_ = drive_model_.ForwardKinematics(
      Eigen::Vector2d(input->left_velocity(), input->right_velocity()));

  integrated_heading_ += delta_heading;
  cartesian_position_(0) += std::cos(integrated_heading_) * delta_linear;
  cartesian_position_(1) += std::sin(integrated_heading_) * delta_linear;

  if (!goal_reader_.ReadLastMessage(&goal)) {
    // If there isn't a goal, we're done -- all we need is odometry
    return;
  }

  bool in_closed_loop = goal->has_path_goal() && driver_station->is_sys_active();
  if (in_closed_loop) {
    closed_loop_.SetGoal(goal);
    closed_loop_.Update(&output, &status);
  } else {
    open_loop_.SetGoal(goal);
    open_loop_.Update(&output);
  }

  status->set_estimated_x_position(cartesian_position_(0));
  status->set_estimated_y_position(cartesian_position_(1));
  status->set_estimated_heading(integrated_heading_);
  status->set_linear_velocity(linear_angular_velocity_(0));
  status->set_angular_velocity(linear_angular_velocity_(1));
  Eigen::Vector2d predicted_v = drive_model_.ForwardDynamics(
      linear_angular_velocity_,
      Eigen::Vector2d(output->left_setpoint() * 12,
                      output->right_setpoint() * 12),
      true);
  status->set_pred_lv(predicted_v(0));
  status->set_pred_rv(predicted_v(0));

  output->set_high_gear(goal->high_gear());

  status_queue_->WriteMessage(status);
  output_queue_->WriteMessage(output);
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
