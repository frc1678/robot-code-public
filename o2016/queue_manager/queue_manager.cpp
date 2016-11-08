#include "queue_manager.h"

namespace o2016 {

QueueManager& QueueManager::GetInstance() {
  static QueueManager instance;
  return instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>&
QueueManager::pdp_status_queue() {
  return pdp_status_queue_;
}

muan::wpilib::DriverStationQueue& QueueManager::driver_station_queue() {
  return driver_station_queue_;
}

o2016::turret::TurretInputQueue& QueueManager::turret_input_queue() {
  return turret_input_queue_;
}

o2016::turret::TurretGoalQueue& QueueManager::turret_goal_queue() {
  return turret_goal_queue_;
}

o2016::turret::TurretStatusQueue& QueueManager::turret_status_queue() {
  return turret_status_queue_;
}

o2016::turret::TurretOutputQueue& QueueManager::turret_output_queue() {
  return turret_output_queue_;
}

o2016::drivetrain::DrivetrainInputQueue&
QueueManager::drivetrain_input_queue() {
  return drivetrain_input_queue_;
}

o2016::drivetrain::DrivetrainGoalQueue& QueueManager::drivetrain_goal_queue() {
  return drivetrain_goal_queue_;
}

o2016::drivetrain::DrivetrainStatusQueue&
QueueManager::drivetrain_status_queue() {
  return drivetrain_status_queue_;
}

o2016::drivetrain::DrivetrainOutputQueue&
QueueManager::drivetrain_output_queue() {
  return drivetrain_output_queue_;
}

o2016::catapult::CatapultInputQueue& QueueManager::catapult_input_queue() {
  return catapult_input_queue_;
}

o2016::catapult::CatapultGoalQueue& QueueManager::catapult_goal_queue() {
  return catapult_goal_queue_;
}

o2016::catapult::CatapultStatusQueue& QueueManager::catapult_status_queue() {
  return catapult_status_queue_;
}

o2016::catapult::CatapultOutputQueue& QueueManager::catapult_output_queue() {
  return catapult_output_queue_;
}

o2016::intake::IntakeInputQueue& QueueManager::intake_input_queue() {
  return intake_input_queue_;
}

o2016::intake::IntakeGoalQueue& QueueManager::intake_goal_queue() {
  return intake_goal_queue_;
}

o2016::intake::IntakeStatusQueue& QueueManager::intake_status_queue() {
  return intake_status_queue_;
}

o2016::intake::IntakeOutputQueue& QueueManager::intake_output_queue() {
  return intake_output_queue_;
}

o2016::secondaries::SecondariesOutputQueue&
QueueManager::secondaries_output_queue() {
  return secondaries_output_queue_;
}

}  // o2016
