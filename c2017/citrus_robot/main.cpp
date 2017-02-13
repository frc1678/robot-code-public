#include "c2017/citrus_robot/main.h"
#include "WPILib.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace citrus_robot {

CitrusRobot::CitrusRobot()
    : throttle_{1, &QueueManager::GetInstance().throttle_status_queue(),
                muan::teleop::JoystickType::THROTTLE},
      wheel_{0, &QueueManager::GetInstance().wheel_status_queue(), muan::teleop::JoystickType::WHEEL},
      gamepad_{2, &QueueManager::GetInstance().manipulator_status_queue(), muan::teleop::JoystickType::XBOX} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
}

void CitrusRobot::Update() {
  if (DriverStation::GetInstance().IsAutonomous()) {
    lemonscript_.Start();  // Weird to call Start in a loop, but it's a setter so it's fine
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    lemonscript_.Stop();  // Weirder to do this, but it works :/

    // Update joysticks
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    SendDrivetrainMessage();
  }

  SendDSMessage();
}

void CitrusRobot::SendDSMessage() {
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

  c2017::QueueManager::GetInstance().driver_station_queue()->WriteMessage(status);
}

void CitrusRobot::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  c2017::QueueManager::GetInstance().drivetrain_goal_queue()->WriteMessage(drivetrain_goal);
}

}  // namespace citrus_robot
}  // namespace c2017
