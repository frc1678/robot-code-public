#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

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

  logger_.AddQueue("wpilib_output", &superstructure_output_queue_);
  logger_.AddQueue("superstructure_status", &superstructure_status_queue_);

  logger_.AddQueue("shooter_input", &shooter_input_queue_);
  logger_.AddQueue("shooter_status", &shooter_status_queue_);

  logger_.AddQueue("magazine_status", &magazine_status_queue_);

  logger_.AddQueue("ground_gear_input", &ground_gear_input_queue_);
  logger_.AddQueue("ground_gear_status", &ground_gear_status_queue_);

  logger_.AddQueue("ground_ball_intake_status", &ground_ball_intake_status_queue_);

  logger_.AddQueue("climber_input", &climber_input_queue_);
  logger_.AddQueue("climber_status", &climber_status_queue_);

  logger_.AddQueue("vision_input_queue", &vision_input_queue_);
  logger_.AddQueue("vision_status_queue", &vision_status_queue_);
  logger_.AddQueue("vision_goal_queue", &vision_goal_queue_);

  logger_.AddQueue("intake_group_goal", &intake_group_goal_queue_);
  logger_.AddQueue("shooter_group_goal", &shooter_group_goal_queue_);

  logger_.AddQueue("manipulator_status", &manipulator_status_queue_);
  logger_.AddQueue("wheel_status", &wheel_status_queue_);
  logger_.AddQueue("throttle_status", &throttle_status_queue_);
  logger_.AddQueue("xbox_rumble", &xbox_rumble_queue_);

  // Webdash
  webdash_.AddQueue("pdp_status", &pdp_status_queue_);
  webdash_.AddQueue("driver_station", &driver_station_queue_);
  webdash_.AddQueue("gyro", &gyro_queue_);

  webdash_.AddQueue("drivetrain_input", &drivetrain_input_queue_);
  webdash_.AddQueue("drivetrain_goal", &drivetrain_goal_queue_);
  webdash_.AddQueue("drivetrain_status", &drivetrain_status_queue_);
  webdash_.AddQueue("drivetrain_output", &drivetrain_output_queue_);

  webdash_.AddQueue("wpilib_output", &superstructure_output_queue_);
  webdash_.AddQueue("superstructure_status", &superstructure_status_queue_);

  webdash_.AddQueue("shooter_input", &shooter_input_queue_);
  webdash_.AddQueue("shooter_status", &shooter_status_queue_);

  webdash_.AddQueue("magazine_status", &magazine_status_queue_);

  webdash_.AddQueue("ground_gear_input", &ground_gear_input_queue_);
  webdash_.AddQueue("ground_gear_status", &ground_gear_status_queue_);

  webdash_.AddQueue("ground_ball_intake_status", &ground_ball_intake_status_queue_);

  webdash_.AddQueue("climber_input", &climber_input_queue_);
  webdash_.AddQueue("climber_status", &climber_status_queue_);

  webdash_.AddQueue("vision_input_queue", &vision_input_queue_);
  webdash_.AddQueue("vision_status_queue", &vision_status_queue_);
  webdash_.AddQueue("vision_goal_queue", &vision_goal_queue_);

  webdash_.AddQueue("intake_group_goal", &intake_group_goal_queue_);
  webdash_.AddQueue("shooter_group_goal", &shooter_group_goal_queue_);

  webdash_.AddQueue("manipulator_status", &manipulator_status_queue_);
  webdash_.AddQueue("wheel_status", &wheel_status_queue_);
  webdash_.AddQueue("throttle_status", &throttle_status_queue_);
  webdash_.AddQueue("xbox_rumble", &xbox_rumble_queue_);

  std::thread webdash_thread{std::ref(webdash_)};
  webdash_thread.detach();
#endif  // FRC1678_NO_QUEUE_LOGGING
}

QueueManager* QueueManager::GetInstance() {
  static QueueManager instance;
  return &instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 512>>* QueueManager::pdp_status_queue() {
  return &pdp_status_queue_;
}

muan::wpilib::DriverStationQueue* QueueManager::driver_station_queue() { return &driver_station_queue_; }

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

c2017::wpilib::WpilibOutputQueue* QueueManager::superstructure_output_queue() {
  return &superstructure_output_queue_;
}

c2017::superstructure::SuperstructureStatusQueue* QueueManager::superstructure_status_queue() {
  return &superstructure_status_queue_;
}

// Shooter
c2017::shooter::ShooterInputQueue* QueueManager::shooter_input_queue() { return &shooter_input_queue_; }

c2017::shooter::ShooterStatusQueue* QueueManager::shooter_status_queue() { return &shooter_status_queue_; }

c2017::magazine::MagazineStatusQueue* QueueManager::magazine_status_queue() {
  return &magazine_status_queue_;
}

// Ground Gear Intake
c2017::ground_gear_intake::GroundGearIntakeInputQueue* QueueManager::ground_gear_input_queue() {
  return &ground_gear_input_queue_;
}

c2017::ground_gear_intake::GroundGearIntakeOutputQueue* QueueManager::ground_gear_output_queue() {
  return &ground_gear_output_queue_;
}

c2017::ground_gear_intake::GroundGearIntakeStatusQueue* QueueManager::ground_gear_status_queue() {
  return &ground_gear_status_queue_;
}

// Ground Ball Intake
c2017::ground_ball_intake::GroundBallIntakeStatusQueue* QueueManager::ground_ball_intake_status_queue() {
  return &ground_ball_intake_status_queue_;
}

// Climber
c2017::climber::ClimberInputQueue* QueueManager::climber_input_queue() { return &climber_input_queue_; }

c2017::climber::ClimberStatusQueue* QueueManager::climber_status_queue() { return &climber_status_queue_; }

// Vision
c2017::vision::VisionInputQueue* QueueManager::vision_input_queue() { return &vision_input_queue_; }

c2017::vision::VisionStatusQueue* QueueManager::vision_status_queue() { return &vision_status_queue_; }

c2017::vision::VisionGoalQueue* QueueManager::vision_goal_queue() { return &vision_goal_queue_; }

// Group Goal Queues
c2017::intake_group::IntakeGroupGoalQueue* QueueManager::intake_group_goal_queue() {
  return &intake_group_goal_queue_;
}

c2017::shooter_group::ShooterGroupGoalQueue* QueueManager::shooter_group_goal_queue() {
  return &shooter_group_goal_queue_;
}

c2017::lights::LightsOutputQueue* QueueManager::lights_output_queue() { return &lights_output_queue_; }

//  Joystick Queues
muan::teleop::JoystickStatusQueue* QueueManager::manipulator_status_queue() {
  return &manipulator_status_queue_;
}

muan::teleop::JoystickStatusQueue* QueueManager::wheel_status_queue() { return &wheel_status_queue_; }

muan::teleop::JoystickStatusQueue* QueueManager::throttle_status_queue() { return &throttle_status_queue_; }

muan::teleop::XBoxRumbleQueue* QueueManager::xbox_rumble_queue() { return &xbox_rumble_queue_; }

void QueueManager::Reset() {
  pdp_status_queue_.Reset();
  driver_station_queue_.Reset();

  gyro_queue_.Reset();

  drivetrain_goal_queue_.Reset();
  drivetrain_input_queue_.Reset();
  drivetrain_output_queue_.Reset();
  drivetrain_status_queue_.Reset();

  superstructure_output_queue_.Reset();

  shooter_input_queue_.Reset();
  shooter_status_queue_.Reset();

  magazine_status_queue_.Reset();

  ground_gear_input_queue_.Reset();
  ground_gear_status_queue_.Reset();

  ground_ball_intake_status_queue_.Reset();

  vision_input_queue_.Reset();
  vision_status_queue_.Reset();
  vision_goal_queue_.Reset();

  climber_input_queue_.Reset();
  climber_status_queue_.Reset();

  intake_group_goal_queue_.Reset();
  shooter_group_goal_queue_.Reset();

  lights_output_queue_.Reset();

  manipulator_status_queue_.Reset();
  wheel_status_queue_.Reset();
  throttle_status_queue_.Reset();
  xbox_rumble_queue_.Reset();
}
}  // namespace c2017
