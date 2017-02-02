#include "c2017/wpilib_update/main.h"
#include "WPILib.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace citrus_robot {

CitrusRobot::CitrusRobot() : throttle_{1}, wheel_{0}, gamepad_{2} {
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
  }

  SendDSMessage();
  SendDrivetrainMessage();
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
    return;  /// TODO: this is only to allow vision to run
    frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

    double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
    double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
    bool quickturn = quickturn_->is_pressed();

    drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
    drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
    drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

    c2017::QueueManager::GetInstance()
        .drivetrain_goal_queue()
        ->WriteMessage(drivetrain_goal);
}

}  // namespace citrus_robot
}  // namespace c2017
