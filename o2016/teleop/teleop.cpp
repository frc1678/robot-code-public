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
}

void Teleop::Update() {
  // Update joysticks
  throttle_.Update();
  wheel_.Update();
  gamepad_.Update();

  SendDSMessage();
  SendDrivetrainMessage();
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
  status->set_brownout(RobotController::IsBrownedOut());
  status->set_has_ds_connection(DriverStation::GetInstance().IsDSAttached());

  o2016::QueueManager::GetInstance().driver_station_queue().WriteMessage(status);
}

void Teleop::SendDrivetrainMessage() {
  o2016::drivetrain::StackDrivetrainGoal drivetrain_goal;

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

  o2016::QueueManager::GetInstance().drivetrain_goal_queue().WriteMessage(drivetrain_goal);
}

}  // namespace teleop
}  // namespace o2016
