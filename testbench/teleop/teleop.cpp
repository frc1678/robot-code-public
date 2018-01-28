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

Teleop::Teleop()
    : properties_{1, 1, 1, 1,
                  testbench::drivetrain::GetDrivetrainConfig().robot_radius},
      throttle_{1},
      wheel_{0},
      ds_sender_{QueueManager::GetInstance()->driver_station_queue()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
}

void Teleop::Update() {
  // Update joysticks
  throttle_.Update();
  wheel_.Update();

  ds_sender_.Send();

  SendDrivetrainMessage();
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

  drivetrain_goal->set_gear(
      high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                 : frc971::control_loops::drivetrain::Gear::kLowGear);

  testbench::QueueManager::GetInstance()->drivetrain_goal_queue()->WriteMessage(
      drivetrain_goal);
}

}  // namespace teleop
}  // namespace testbench
