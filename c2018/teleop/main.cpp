#include "c2018/teleop/main.h"

#include <string>

#include "WPILib.h"
#include "muan/logging/logger.h"
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
using c2018::climber::ClimberGoalProto;
using c2018::score_subsystem::ScoreSubsystemGoalProto;

TeleopBase::TeleopBase()
    : throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      climber_goal_queue_{QueueManager<ClimberGoalProto>::Fetch()},
      score_subsystem_goal_queue_{
          QueueManager<ScoreSubsystemGoalProto>::Fetch()} {
  initialize_climb_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  climb_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));
  stop_climb_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));
  godmode_ = gamepad_.MakeButton(
      uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));  // TODO(hanson/gemma/ellie)
                                                     // add godmodes for
                                                     // intaking/outtaking
  intake_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  stow_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  outtake_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  score_back_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  score_front_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));

  height_0_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  height_1_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);
  height_2_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);

  godmode_up_ = gamepad_.MakeAxis(5, -.7);   // Right Joystick North
  godmode_down_ = gamepad_.MakeAxis(5, .7);  // Right Joystick South

  top_mode_ = gamepad_.MakeAxis(1, -.7);    // Left Joystick North
  bottom_mode_ = gamepad_.MakeAxis(1, .7);  // Left Joystick South

  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);

  quickturn_ = wheel_.MakeButton(5);

  // Default values
  climber_goal_->set_climber_goal(c2018::climber::NONE);

  score_subsystem_goal_->set_score_goal(c2018::score_subsystem::SCORE_NONE);
  score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::INTAKE_NONE);
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
    SendScoreSubsystemMessage();
    SendClimbSubsystemMessage();
  }
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

void TeleopBase::SendScoreSubsystemMessage() {
  score_subsystem_goal_->set_score_goal(c2018::score_subsystem::SCORE_NONE);
  score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::INTAKE_NONE);

  // Godmode
  if (godmode_->is_pressed()) {
    if (godmode_up_->is_pressed()) {
      // logic
    } else if (godmode_down_->is_pressed()) {
      // more logic
    }
    // room for more godmode buttons if needed
  }

  // Elevator heights + intakes
  if (height_0_->is_pressed()) {
    score_subsystem_goal_->set_score_goal(c2018::score_subsystem::INTAKE_0);
  } else if (height_1_->is_pressed()) {
    score_subsystem_goal_->set_score_goal(c2018::score_subsystem::INTAKE_1);
  } else if (height_2_->is_pressed()) {
    score_subsystem_goal_->set_score_goal(c2018::score_subsystem::INTAKE_2);
  }

  // Intake modes
  if (intake_->is_pressed()) {
    score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::INTAKE);
  } else if (intake_->was_released()) {
    score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::FORCE_STOP);
  }

  if (stow_->was_clicked()) {
    score_subsystem_goal_->set_score_goal(c2018::score_subsystem::FORCE_STOW);
  }

  if (outtake_->is_pressed()) {
    score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::OUTTAKE);
  } else if (outtake_->was_released()) {
    score_subsystem_goal_->set_intake_goal(c2018::score_subsystem::FORCE_STOP);
  }

  // Scoring modes
  if (score_front_->is_pressed()) {
    if (top_mode_->is_pressed()) {
      score_subsystem_goal_->set_score_goal(
          c2018::score_subsystem::SCALE_HIGH_FORWARD);
    } else if (bottom_mode_->is_pressed()) {
      score_subsystem_goal_->set_score_goal(c2018::score_subsystem::SWITCH);
    } else {
      score_subsystem_goal_->set_score_goal(
          c2018::score_subsystem::SCALE_MID_FORWARD);
    }
  }
  if (score_back_->is_pressed()) {
    if (top_mode_->is_pressed()) {
      score_subsystem_goal_->set_score_goal(
          c2018::score_subsystem::SCALE_HIGH_REVERSE);
    } else if (bottom_mode_->is_pressed()) {
      score_subsystem_goal_->set_score_goal(c2018::score_subsystem::EXCHANGE);
    } else {
      score_subsystem_goal_->set_score_goal(
          c2018::score_subsystem::SCALE_MID_REVERSE);
    }
  }

  score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal_);
}

void TeleopBase::SendClimbSubsystemMessage() {
  if (initialize_climb_->was_clicked()) {
    climber_goal_->set_climber_goal(c2018::climber::APPROACHING);
    if (climb_->was_clicked()) {
      climber_goal_->set_climber_goal(c2018::climber::CLIMBING);
    }
  } else if (stop_climb_->was_clicked()) {
    climber_goal_->set_climber_goal(c2018::climber::NONE);
  }

  climber_goal_queue_->WriteMessage(climber_goal_);
}

}  // namespace teleop
}  // namespace c2018
