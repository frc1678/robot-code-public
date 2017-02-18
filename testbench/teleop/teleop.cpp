#include "testbench/teleop/teleop.h"
#include "muan/wpilib/queue_types.h"
#include "testbench/queue_manager/queue_manager.h"
#include "testbench/subsystems/drivetrain/drivetrain_base.h"

namespace testbench {
namespace teleop {

using frc971::control_loops::drivetrain::GoalProto;
using frc971::control_loops::drivetrain::InputProto;
using frc971::control_loops::drivetrain::StatusProto;
using frc971::control_loops::drivetrain::OutputProto;

Teleop::Teleop() {
    : properties_{1, 1, 1, 1, testbench::drivetrain::GetDrivetrainConfig().robot_radius},
      throttle_{1},
      wheel_{0},
  shifting_low_ = throttle_.GetButton(4);
  shifting_high_ = throttle_.GetButton(5);
  quickturn_ = wheel_.GetButton(5);
}

void Teleop::Update() {
  // Update joysticks
  throttle_.Update();
  wheel_.Update();

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
  status->set_brownout(DriverStation::GetInstance().IsBrownedOut());
  status->set_has_ds_connection(DriverStation::GetInstance().IsDSAttached());

  testbench::QueueManager::GetInstance()->driver_station_queue()->WriteMessage(status);
}

void Teleop::SendDrivetrainMessage() {
  frc971::control_loops::drivetrain::GoalProto drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  }

  if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  drivetrain_goal->set_gear(high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                                       : frc971::control_loops::drivetrain::Gear::kLowGear);

  testbench::QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(drivetrain_goal);
}

}  // namespace teleop
}  // namespace testbench
