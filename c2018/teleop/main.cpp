#include <string>
#include "WPILib.h"
#include "c2018/teleop/main.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2018 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;

TeleopBase::TeleopBase()
    : throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);
  intake_ = gamepad_.MakeButton(1);
  outtake_ = gamepad_.MakeButton(2);
  prep_low_ = gamepad_.MakeButton(3);
  prep_mid_ = throttle_.MakeButton(10);
  prep_mid_back_ = throttle_.MakeButton(11);
  prep_high_ = gamepad_.MakeButton(4);
  prep_high_back_ = gamepad_.MakeButton(6);
  idle_bottom_ = gamepad_.MakeButton(5);
  intake_h0_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  intake_h1_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);
  intake_h2_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);
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

  if (intake_h0_->was_clicked()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::ScoreGoal::HEIGHT_0);
  }

  if (intake_h1_->was_clicked()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::ScoreGoal::HEIGHT_1);
  }

  if (intake_h2_->was_clicked()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::ScoreGoal::HEIGHT_2);
  }

  if (prep_mid_->was_clicked()) {
    score_goal_proto_->set_score_goal(
        c2018::score_subsystem::ScoreGoal::PREP_SCORE_MID);
    std::cout << "button detected" << std::endl;
  }

  if (prep_mid_back_->was_clicked()) {
    score_goal_proto_->set_score_goal(
        c2018::score_subsystem::ScoreGoal::PREP_SCORE_MID_BACK);
    std::cout << "button detected" << std::endl;
  }

  if (prep_high_->was_clicked()) {
    score_goal_proto_->set_score_goal(
        c2018::score_subsystem::ScoreGoal::PREP_SCORE_HIGH);
    std::cout << "button detected" << std::endl;
  }

  if (prep_high_back_->was_clicked()) {
    score_goal_proto_->set_score_goal(
        c2018::score_subsystem::ScoreGoal::PREP_SCORE_HIGH_BACK);
    std::cout << "button detected" << std::endl;
  }

  if (idle_bottom_->was_clicked()) {
    score_goal_proto_->set_score_goal(
        c2018::score_subsystem::ScoreGoal::IDLE_BOTTOM);
    std::cout << "button detected" << std::endl;
  }

  if (prep_low_->was_clicked()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::PREP_SCORE_LOW);
    std::cout << "button detected" << std::endl;
  }

  if (intake_->is_pressed()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::INTAKE_LOW);
  } else if (outtake_->is_pressed()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::OUTTAKE_LOW);
  }
  
  if (intake_->was_released() || outtake_->was_released()) {
    score_goal_proto_->set_score_goal(c2018::score_subsystem::IDLE_LOW);
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

  drivetrain_goal_queue_->WriteMessage(drivetrain_goal);
}

}  // namespace teleop
}  // namespace c2018
