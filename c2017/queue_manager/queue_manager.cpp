#include "queue_manager.h"

namespace c2017 {

QueueManager& QueueManager::GetInstance() {
  static QueueManager instance;
  return instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& QueueManager::pdp_status_queue() {
  return pdp_status_queue_;
}

muan::wpilib::DriverStationQueue*
QueueManager::driver_station_queue() {
  return &driver_station_queue_;
}

muan::wpilib::gyro::GyroQueue*
QueueManager::gyro_queue() {
  return &gyro_queue_;
}

frc971::control_loops::drivetrain::InputQueue*
QueueManager::drivetrain_input_queue() {
  return &drivetrain_input_queue_;
}

frc971::control_loops::drivetrain::GoalQueue*
QueueManager::drivetrain_goal_queue() {
  return &drivetrain_goal_queue_;
}

frc971::control_loops::drivetrain::StatusQueue*
QueueManager::drivetrain_status_queue() {
  return &drivetrain_status_queue_;
}

frc971::control_loops::drivetrain::OutputQueue*
QueueManager::drivetrain_output_queue() {
  return &drivetrain_output_queue_;
}

shooter::ShooterStatusQueue&
QueueManager::shooter_status_queue() {
  return shooter_status_queue_;
}

}  // c2017
