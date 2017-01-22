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

c2017::wpilib::WpilibOutputQueue& 
QueueManager::superstructure_output_queue() {
  return superstructure_output_queue_;
}

// Shooter
c2017::shooter::ShooterGoalQueue&
QueueManager::shooter_goal_queue() {
  return shooter_goal_queue_;
}

c2017::shooter::ShooterInputQueue&
QueueManager::shooter_input_queue() {
  return shooter_input_queue_;
}

c2017::shooter::ShooterOutputQueue&
QueueManager::shooter_output_queue() {
  return shooter_output_queue_;
}

c2017::shooter::ShooterStatusQueue&
QueueManager::shooter_status_queue() {
  return shooter_status_queue_;
}

// Magazine
c2017::magazine::MagazineGoalQueue&
QueueManager::magazine_goal_queue() {
  return magazine_goal_queue_;
}

c2017::magazine::MagazineInputQueue&
QueueManager::magazine_input_queue() {
  return magazine_input_queue_;
}

c2017::magazine::MagazineOutputQueue&
QueueManager::magazine_output_queue() {
  return magazine_output_queue_;
}

c2017::magazine::MagazineStatusQueue&
QueueManager::magazine_status_queue() {
  return magazine_status_queue_;
}

// Ground Gear Intake
c2017::ground_gear_intake::GroundGearIntakeGoalQueue&
QueueManager::ground_gear_goal_queue() {
  return ground_gear_goal_queue_;
}

c2017::ground_gear_intake::GroundGearIntakeInputQueue&
QueueManager::ground_gear_input_queue() {
  return ground_gear_input_queue_;
}

c2017::ground_gear_intake::GroundGearIntakeOutputQueue&
QueueManager::ground_gear_output_queue() {
  return ground_gear_output_queue_;
}

c2017::ground_gear_intake::GroundGearIntakeStatusQueue&
QueueManager::ground_gear_status_queue() {
  return ground_gear_status_queue_;
}

// Ground Ball Intake
c2017::ball_intake::BallIntakeGoalQueue&
QueueManager::ball_intake_goal_queue() {
  return ball_intake_goal_queue_;
}

c2017::ball_intake::BallIntakeOutputQueue&
QueueManager::ball_intake_output_queue() {
  return ball_intake_output_queue_;
}

c2017::ball_intake::BallIntakeStatusQueue&
QueueManager::ball_intake_status_queue() {
  return ball_intake_status_queue_;
}

// Climber
c2017::climber::ClimberGoalQueue&
QueueManager::climber_goal_queue() {
  return climber_goal_queue_;
}

c2017::climber::ClimberInputQueue&
QueueManager::climber_input_queue() {
  return climber_input_queue_;
}

c2017::climber::ClimberOutputQueue&
QueueManager::climber_output_queue() {
  return climber_output_queue_;
}

c2017::climber::ClimberStatusQueue&
QueueManager::climber_status_queue() {
  return climber_status_queue_;
}

// Group Goal Queues
c2017::intake_group::IntakeGroupGoalQueue&
QueueManager::intake_group_goal_queue() {
  return intake_group_goal_queue_;
}

c2017::shooter_group::ShooterGroupGoalQueue&
QueueManager::shooter_group_goal_queue() {
  return shooter_group_goal_queue_;
}

// Trigger Queues
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
