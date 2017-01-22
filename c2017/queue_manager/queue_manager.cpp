#include "queue_manager.h"
#include "c2017/subsystems/superstructure/trigger/queue_types.h"

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

ground_ball_intake::GroundBallIntakeStatusQueue& QueueManager::ground_ball_intake_status_queue() {
  return ground_ball_intake_status_queue_;
}

ground_ball_intake::GroundBallIntakeOutputQueue& QueueManager::ground_ball_intake_output_queue() {
  return ground_ball_intake_output_queue_;
}

ground_ball_intake::GroundBallIntakeGoalQueue& QueueManager::ground_ball_intake_goal_queue() {
  return ground_ball_intake_goal_queue_;
}

c2017::trigger::TriggerGoalQueue& QueueManager::trigger_goal_queue() {
  return trigger_goal_queue_;
}

c2017::trigger::TriggerInputQueue& QueueManager::trigger_input_queue() {
  return trigger_input_queue_;
}

c2017::trigger::TriggerOutputQueue& QueueManager::trigger_output_queue() {
  return trigger_output_queue_;
}

c2017::trigger::TriggerStatusQueue& QueueManager::trigger_status_queue() {
  return trigger_status_queue_;
}

}  // c2017
