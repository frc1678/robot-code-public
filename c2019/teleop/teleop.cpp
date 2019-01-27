#include "c2019/teleop/teleop.h"
#include "c2019/commands/drive_straight.h"
#include "c2019/commands/test_auto.h"
#include "muan/logging/logger.h"

namespace c2019 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using c2019::commands::Command;
using commands::AutoGoalProto;
using commands::AutoStatusProto;

TeleopBase::TeleopBase()
    : ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      auto_status_reader_{QueueManager<AutoStatusProto>::Fetch()->MakeReader()},
      auto_goal_queue_{QueueManager<AutoGoalProto>::Fetch()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
  // TODO(jishnu) change these buttons to whatever Nathan wants
  exit_auto_ = throttle_.MakeButton(6);
  test_auto_ = throttle_.MakeButton(7);
  drive_straight_ = throttle_.MakeButton(8);
}

void TeleopBase::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("TeleopBase");

  LOG(INFO, "Starting TeleopBase thread!");

  running_ = true;
  while (running_) {
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
    Update();
    phased_loop.SleepUntilNext();
  }
}

void TeleopBase::Stop() { running_ = false; }

void TeleopBase::Update() {
  AutoStatusProto auto_status;
  AutoGoalProto auto_goal;

  auto_status_reader_.ReadLastMessage(&auto_status);

  if (RobotController::IsSysActive() && !auto_status->running_command()) {
    SendDrivetrainMessage();
  }

  if (exit_auto_->was_clicked()) {
    auto_goal->set_run_command(false);
    auto_goal_queue_->WriteMessage(auto_goal);
  } else if (!auto_status->running_command()) {
    if (test_auto_->was_clicked()) {
      auto_goal->set_run_command(true);
      auto_goal->set_command(Command::TEST_AUTO);
    } else if (drive_straight_->was_clicked()) {
      auto_goal->set_run_command(true);
      auto_goal->set_command(Command::DRIVE_STRAIGHT);
    } else {
      auto_goal->set_run_command(false);
      auto_goal->set_command(Command::NONE);
    }

    auto_goal_queue_->WriteMessage(auto_goal);

    // TODO(jishnu) add actual commands
    // NOTE: not using a switch here due to cross-initialization of the threads
    if (auto_goal->command() == Command::DRIVE_STRAIGHT) {
      commands::DriveStraight drive_straight_command;
      std::thread drive_straight_thread(drive_straight_command);
      drive_straight_thread.detach();
    } else if (auto_goal->command() == Command::TEST_AUTO) {
      commands::TestAuto test_auto_command;
      std::thread test_auto_thread(test_auto_command);
      test_auto_thread.detach();
    }
  }

  ds_sender_.Send();
}

void TeleopBase::SendDrivetrainMessage() {
  DrivetrainGoal drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  // Shifting gears
  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  }
  if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  drivetrain_goal->set_high_gear(high_gear_);

  // Drive controls
  drivetrain_goal->mutable_teleop_goal()->set_steering(-wheel);
  drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

}  // namespace teleop
}  // namespace c2019
