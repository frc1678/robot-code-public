#include "o2016/teleop/teleop.h"
#include "WPILib.h"
#include "muan/wpilib/queue_types.h"
#include "o2016/queue_manager/queue_manager.h"

namespace o2016 {

namespace teleop {

Teleop::Teleop() : throttle_{1}, wheel_{0}, gamepad_{2} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = throttle_.MakeButton(5);

  intake_ = gamepad_.MakeButton(6); // R bumper?
  spit_ = gamepad_.MakeButton(5); // L bumper?
  defense_ = gamepad_.MakeButton(1); // A?
  prep_shot_ = gamepad_.MakeButton(2); // B?
  vision_fail_toggle_ = gamepad_.MakeButton(7); // L joystick press?

  shoot_ = throttle_.MakeButton(3); // Center button?

  Update();
}

void Teleop::Update() {
  // Update joysticks
  throttle_.Update();
  wheel_.Update();
  gamepad_.Update();

  SendDSMessage();
  SendDrivetrainMessage();
  SendSuperstructureMessage();
}

void Teleop::SendDSMessage() {
  muan::wpilib::DriverStationProto status;

  if (DriverStation::GetInstance().IsDisabled()) {
    status->set_mode(RobotMode::DISABLED);
  } else if (DriverStation::GetInstance().IsAutonomous()) {
    status->set_mode(RobotMode::AUTONOMOUS);
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    status->set_mode(RobotMode::TELEOP);
  } else {
    status->set_mode(RobotMode::ESTOP);
  }

  status->set_battery_voltage(DriverStation::GetInstance().GetBatteryVoltage());
  status->set_brownout(DriverStation::GetInstance().IsBrownedOut());
  status->set_has_ds_connection(DriverStation::GetInstance().IsDSAttached());

  o2016::QueueManager::GetInstance().driver_station_queue().WriteMessage(
      status);
}

void Teleop::SendDrivetrainMessage() {
  o2016::drivetrain::DrivetrainGoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  double angular;
  double forward = throttle;

  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  } else if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  if (quickturn) {
    angular = (high_gear_ ? 5 : 3) * wheel;
  } else {
    angular = (high_gear_ ? 5 : 3) * throttle * wheel;
  }

  drivetrain_goal->mutable_velocity_command()->set_forward_velocity(forward);
  drivetrain_goal->mutable_velocity_command()->set_angular_velocity(angular);
  drivetrain_goal->set_gear(high_gear_ ? o2016::drivetrain::Gear::kHighGear
                                       : o2016::drivetrain::Gear::kLowGear);

  o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(
      drivetrain_goal);
}

void Teleop::SendSuperstructureMessage() {
  o2016::SuperstructureGoalProto goal;

  if (intake_->was_clicked()) {
    goal->set_goal_state(o2016::superstructure::INTAKE);
  }

  if (spit_->was_clicked()) {
    goal->set_goal_state(o2016::superstructure::SPIT);
  }

  if (defense_->was_clicked()) {
    goal->set_goal_state(o2016::superstructure::IDLE);
  }

  if (prep_shot_->was_clicked()) {
    goal->set_goal_state(o2016::superstructure::AIMING);
  }

  if (shoot_->was_clicked()) {
    goal->set_goal_state(o2016::superstructure::FIRING);
  }

  if (vision_fail_toggle_->was_clicked()) {
    vision_fail_ = !vision_fail_;
  }

  goal->set_vision_override(vision_fail_);

  o2016::QueueManager::GetInstance().superstructure_goal_queue().WriteMessage(goal);
}

}  // teleop

}  // o2016
