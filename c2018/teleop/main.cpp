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
using c2018::score_subsystem::ScoreSubsystemStatusProto;

TeleopBase::TeleopBase()
    : throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      climber_goal_queue_{QueueManager<ClimberGoalProto>::Fetch()},
      score_subsystem_goal_queue_{
          QueueManager<ScoreSubsystemGoalProto>::Fetch()},
      score_subsystem_status_queue_{
          QueueManager<ScoreSubsystemStatusProto>::Fetch()} {
  hook_up_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  batter_down_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));
  godmode_ = gamepad_.MakeButton(
      uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));  // TODO(hanson/gemma/ellie)
                                                     // add godmodes for
                                                     // intaking/outtaking
  height_0_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  height_1_ = gamepad_.MakePov(0, muan::teleop::Pov::kEast);
  height_2_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);
  height_portal_ = gamepad_.MakePov(0, muan::teleop::Pov::kWest);

  low_ = gamepad_.MakeAxisRange(136, 225, 0, 1, 0.7);
  front_ = gamepad_.MakeAxisRange(15, 135, 0, 1, 0.7);
  back_ = gamepad_.MakeAxisRange(226, 345, 0, 1, 0.7);

  intake_ = gamepad_.MakeAxis(3, 0.3);
  settle_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  intake_open_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  intake_close_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));

  outtake_slow_ = gamepad_.MakeAxis(2, 0.7);
  outtake_fast_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));

  pos_0_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  pos_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  pos_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  pos_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));

  godmode_up_ = gamepad_.MakeAxis(5, -.7);   // Right Joystick North
  godmode_down_ = gamepad_.MakeAxis(5, .7);  // Right Joystick South

  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);

  quickturn_ = wheel_.MakeButton(5);

  // Default values
  climber_goal_->set_climber_goal(c2018::climber::NONE);
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
  if (DriverStation::GetInstance().IsOperatorControl()) {
    SendDrivetrainMessage();
    SendScoreSubsystemMessage();
    SendClimbSubsystemMessage();
  }
  SetReadableLogName();

  ScoreSubsystemStatusProto score_status;
  score_subsystem_status_queue_->ReadLastMessage(&score_status);
  if (score_status->has_cube() && !had_cube_) {
    rumble_ticks_left_ = kNumRumbleTicks;
  }
  had_cube_ = score_status->has_cube();

  if (rumble_ticks_left_ > 0) {
    // Set rumble on
    rumble_ticks_left_--;
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 1.0);
  } else {
    // Set rumble off
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 0.0);
  }

  ds_sender_.Send();
}

void TeleopBase::SetReadableLogName() {
  /*
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
  */
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
  ScoreSubsystemGoalProto score_subsystem_goal;
  score_subsystem_goal->set_score_goal(c2018::score_subsystem::SCORE_NONE);
  score_subsystem_goal->set_intake_goal(c2018::score_subsystem::INTAKE_NONE);

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
    score_subsystem_goal->set_score_goal(c2018::score_subsystem::INTAKE_0);
  } else if (height_1_->is_pressed()) {
    score_subsystem_goal->set_score_goal(c2018::score_subsystem::INTAKE_1);
  } else if (height_2_->is_pressed()) {
    score_subsystem_goal->set_score_goal(c2018::score_subsystem::INTAKE_2);
  } else if (height_portal_->is_pressed()) {
    score_subsystem_goal->set_score_goal(c2018::score_subsystem::PORTAL);
  }

  // Intake modes
  if (intake_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::INTAKE);
  } else if (intake_open_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::INTAKE_OPEN);
  } else if (intake_close_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::INTAKE_CLOSE);
  } else if (outtake_fast_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::OUTTAKE_FAST);
  } else if (outtake_slow_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::OUTTAKE_SLOW);
  } else if (settle_->is_pressed()) {
    score_subsystem_goal->set_intake_goal(c2018::score_subsystem::SETTLE);
  }

  // Scoring modes
  if (low_->is_pressed()) {
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(c2018::score_subsystem::EXCHANGE);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(c2018::score_subsystem::SWITCH);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(c2018::score_subsystem::STOW);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(c2018::score_subsystem::SCALE_SHOOT);
    }
  } else if (front_->is_pressed()) {
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_LOW_FORWARD);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_MID_FORWARD);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_HIGH_FORWARD);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_SUPER_HIGH_FORWARD);
    }
  } else if (back_->is_pressed()) {
    if (pos_0_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_LOW_REVERSE);
    } else if (pos_1_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_MID_REVERSE);
    } else if (pos_2_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_HIGH_REVERSE);
    } else if (pos_3_->is_pressed()) {
      score_subsystem_goal->set_score_goal(
          c2018::score_subsystem::SCALE_SUPER_HIGH_REVERSE);
    }
  }
  score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal);
}

void TeleopBase::SendClimbSubsystemMessage() {
  if (hook_up_->is_pressed() && !batter_down_->is_pressed()) {
    climber_goal_->set_climber_goal(c2018::climber::APPROACHING);
  } else if (!hook_up_->is_pressed() && batter_down_->is_pressed()) {
    climber_goal_->set_climber_goal(c2018::climber::BATTERING);
  } else if (hook_up_->is_pressed() && batter_down_->is_pressed()) {
    climber_goal_->set_climber_goal(c2018::climber::CLIMBING);
  } else {
    climber_goal_->set_climber_goal(c2018::climber::NONE);
  }

  climber_goal_queue_->WriteMessage(climber_goal_);
}

}  // namespace teleop
}  // namespace c2018
