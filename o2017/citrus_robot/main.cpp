#include "o2017/citrus_robot/main.h"
#include "WPILib.h"
#include "o2017/queue_manager/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace o2017 {

namespace citrus_robot {

CitrusRobot::CitrusRobot()
    : throttle_{1}, wheel_{0}, gamepad_{2}, ds_sender_{&QueueManager::GetInstance().driver_station_queue()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
}

void CitrusRobot::Update() {
  if (DriverStation::GetInstance().IsAutonomous()) {
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    // Update joysticks
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    SendDrivetrainMessage();
  }
  ds_sender_.Send();
}

void CitrusRobot::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  o2017::QueueManager::GetInstance().drivetrain_goal_queue()->WriteMessage(drivetrain_goal);
}

}  // namespace citrus_robot

}  // namespace o2017
