#include "generic_robot/queue_manager/queue_manager.h"

namespace generic_robot {

void QueueManager::StartLogging() {
#ifndef FRC1678_NO_QUEUE_LOGGING
  // Logging
  logger_.AddQueue("pdp_status", &pdp_status_queue_);
  logger_.AddQueue("driver_station", &driver_station_queue_);
  logger_.AddQueue("gyro", &gyro_queue_);

  logger_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  logger_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  logger_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  logger_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  logger_.AddQueue("manipulator_status", &manipulator_status_queue_);
  logger_.AddQueue("wheel_status", &wheel_status_queue_);
  logger_.AddQueue("throttle_status", &throttle_status_queue_);
#endif  // FRC1678_NO_QUEUE_LOGGING
}

QueueManager& QueueManager::GetInstance() {
  static QueueManager instance;
  return instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& QueueManager::pdp_status_queue() {
  return pdp_status_queue_;
}

muan::wpilib::DriverStationQueue& QueueManager::driver_station_queue() { return driver_station_queue_; }

muan::wpilib::gyro::GyroQueue* QueueManager::gyro_queue() { return &gyro_queue_; }

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

  drivetrain_goal_queue_.Reset();
  drivetrain_input_queue_.Reset();
  drivetrain_output_queue_.Reset();
  drivetrain_status_queue_.Reset();
}
}  // namespace generic_robot
