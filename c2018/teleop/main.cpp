#include "c2018/teleop/main.h"
#include <string>
#include "WPILib.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace c2018 {
namespace teleop {

using DrivetrainGoalProto = frc971::control_loops::drivetrain::GoalProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using muan::teleop::JoystickStatusProto;
using muan::queues::QueueManager;

TeleopBase::TeleopBase()
    : throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
  elevator_zero_ = gamepad_.MakeButton(1);
  elevator_one_ = gamepad_.MakeButton(2);
  elevator_two_ = gamepad_.MakeButton(3);
  elevator_score_ = gamepad_.MakeButton(4);
}

void TeleopBase::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("TeleopBase");

  LOG_P("Starting TeleopBase thread!");

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
  if (DriverStation::GetInstance().IsOperatorControl()) {
    SendDrivetrainMessage();
  }

  if (elevator_zero_->was_clicked()) {
    score_goal_proto_->set_elevator_height(0);
  }

  if (elevator_one_->was_clicked()) {
    score_goal_proto_->set_elevator_height(0.3);
  }

  if (elevator_two_->was_clicked()) {
    score_goal_proto_->set_elevator_height(0.6);
  }

  if (elevator_score_->was_clicked()) {
    score_goal_proto_->set_elevator_height(1.8);
  }

  score_goal_queue_->WriteMessage(score_goal_proto_);

  SetReadableLogName();

  ds_sender_.Send();
}

void TeleopBase::SetReadableLogName() {
  if (DriverStation::GetInstance().GetMatchType() !=
          DriverStation::MatchType::kNone &&
      !log_name_set_) {
    std::string name;
    int match_num = DriverStation::GetInstance().GetMatchNumber();
    std::string match_number = std::to_string(match_num);
    // Figure out name for log file
    switch (DriverStation::GetInstance().GetMatchType()) {
      case DriverStation::MatchType::kNone:
        name = "N" + match_number;
        break;
      case DriverStation::MatchType::kPractice:
        name = "P" + match_number;
        break;
      case DriverStation::MatchType::kQualification:
        name = "Q" + match_number;
        break;
      case DriverStation::MatchType::kElimination:
        name = "E" + match_number;
        break;
    }
    muan::logging::FileWriter::CreateReadableName(name);
    log_name_set_ = true;
  }
}

void TeleopBase::SendDrivetrainMessage() {
  using DrivetrainGoal = frc971::control_loops::drivetrain::GoalProto;
  DrivetrainGoal drivetrain_goal;

  double throttle = -throttle_.wpilib_joystick()->GetRawAxis(1);
  double wheel = -wheel_.wpilib_joystick()->GetRawAxis(0);
  bool quickturn = quickturn_->is_pressed();

  if (shifting_high_->was_clicked()) {
    high_gear_ = true;
  }
  if (shifting_low_->was_clicked()) {
    high_gear_ = false;
  }

  drivetrain_goal->set_gear(
      high_gear_ ? frc971::control_loops::drivetrain::Gear::kHighGear
                 : frc971::control_loops::drivetrain::Gear::kLowGear);

  drivetrain_goal->mutable_teleop_command()->set_steering(wheel);
  drivetrain_goal->mutable_teleop_command()->set_throttle(throttle);
  drivetrain_goal->mutable_teleop_command()->set_quick_turn(quickturn);

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

}  // namespace teleop
}  // namespace c2018
