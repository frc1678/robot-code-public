#include "generic_robot/queue_manager/queue_manager.h"

namespace generic_robot {

void QueueManager::StartLogging() {
  // Logging
  logger_.AddQueue("pdp_status", &pdp_status_queue_);
  logger_.AddQueue("driver_station", &driver_station_queue_);
  logger_.AddQueue("gyro", &gyro_queue_);

  logger_.AddQueue("manipulator_status", &manipulator_status_queue_);
  logger_.AddQueue("wheel_status", &wheel_status_queue_);
  logger_.AddQueue("throttle_status", &throttle_status_queue_);
  logger_.AddQueue("xbox_rumble", &xbox_rumble_queue_);

  logger_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  logger_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  logger_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  logger_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  // Webdash
  webdash_.AddQueue("pdp_status", &pdp_status_queue_);
  webdash_.AddQueue("driver_station", &driver_station_queue_);
  webdash_.AddQueue("gyro", &gyro_queue_);

  webdash_.AddQueue("manipulator_status", &manipulator_status_queue_);
  webdash_.AddQueue("wheel_status", &wheel_status_queue_);
  webdash_.AddQueue("throttle_status", &throttle_status_queue_);
  webdash_.AddQueue("xbox_rumble", &xbox_rumble_queue_);

  webdash_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  webdash_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  webdash_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  webdash_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  std::thread webdash_thread{std::ref(webdash_)};
  webdash_thread.detach();

  std::thread logger_thread{std::ref(logger_)};
  logger_thread.detach();
}

QueueManager* QueueManager::GetInstance() {
  static QueueManager instance;
  return &instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>* QueueManager::pdp_status_queue() {
  return &pdp_status_queue_;
}

muan::wpilib::DriverStationQueue* QueueManager::driver_station_queue() {
  return &driver_station_queue_;
}

muan::wpilib::gyro::GyroQueue* QueueManager::gyro_queue() {
  return &gyro_queue_;
}

muan::teleop::JoystickStatusQueue* QueueManager::manipulator_status_queue() {
  return &manipulator_status_queue_;
}

muan::teleop::JoystickStatusQueue* QueueManager::wheel_status_queue() {
  return &wheel_status_queue_;
}

muan::teleop::JoystickStatusQueue* QueueManager::throttle_status_queue() {
  return &throttle_status_queue_;
}

muan::teleop::XBoxRumbleQueue* QueueManager::xbox_rumble_queue() {
  return &xbox_rumble_queue_;
}

frc971::control_loops::drivetrain::InputQueue* QueueManager::drivetrain_input_queue() {
  return &drivetrain_input_queue_;
}

frc971::control_loops::drivetrain::GoalQueue* QueueManager::drivetrain_goal_queue() {
  return &drivetrain_goal_queue_;
}

frc971::control_loops::drivetrain::StatusQueue* QueueManager::drivetrain_status_queue() {
  return &drivetrain_status_queue_;
}

frc971::control_loops::drivetrain::OutputQueue* QueueManager::drivetrain_output_queue() {
  return &drivetrain_output_queue_;
}

void QueueManager::Reset() {
  pdp_status_queue_.Reset();
  driver_station_queue_.Reset();

  gyro_queue_.Reset();

  manipulator_status_queue_.Reset();
  wheel_status_queue_.Reset();
  throttle_status_queue_.Reset();
  xbox_rumble_queue_.Reset();

  drivetrain_goal_queue_.Reset();
  drivetrain_input_queue_.Reset();
  drivetrain_output_queue_.Reset();
  drivetrain_status_queue_.Reset();
}
}  // namespace generic_robot
