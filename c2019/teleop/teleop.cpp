#include "c2019/teleop/teleop.h"

#include <memory>
#include <string>

#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/logging/logger.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace c2019 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::webdash::WebdashProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;
using DrivetrainStatus = muan::subsystems::drivetrain::StatusProto;
using c2019::commands::Command;
using c2019::limelight::LimelightStatusProto;
using c2019::superstructure::SuperstructureGoalProto;
using c2019::superstructure::SuperstructureStatusProto;

using commands::AutoGoalProto;
using commands::AutoStatusProto;

TeleopBase::TeleopBase()
    : superstructure_goal_queue_{QueueManager<
          c2019::superstructure::SuperstructureGoalProto>::Fetch()},
      superstructure_status_queue_{QueueManager<
          c2019::superstructure::SuperstructureStatusProto>::Fetch()},
      webdash_queue_{QueueManager<muan::webdash::WebdashProto>::Fetch()},
      ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")},
      auto_status_reader_{QueueManager<AutoStatusProto>::Fetch()->MakeReader()},
      auto_goal_queue_{QueueManager<AutoGoalProto>::Fetch()} {
  winch_left_ = throttle_.MakeButton(10);
  winch_right_ = throttle_.MakeButton(7);
  drop_forks_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::BACK));
  drop_crawlers_ = gamepad_.MakeAxisRange(-105, -75, 0, 1, 0.8);

  // rezero
  rezero_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::START));

  // scoring positions
  stow_ = gamepad_.MakePov(0, muan::teleop::Pov::kNorth);
  level_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  level_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  level_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  ship_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  // backwards scoring mode
  backwards_ = gamepad_.MakeAxisRange(135, 180, 0, 1, 0.8);
  // intake buttons
  ground_intake_height_ = gamepad_.MakePov(0, muan::teleop::Pov::kSouth);
  cargo_intake_ = gamepad_.MakeAxis(3, 0.3);
  // outtake buttons
  cargo_outtake_ = gamepad_.MakeAxis(2, 0.7);
  hp_hatch_intake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));
  hp_hatch_outtake_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));

  // gear shifting - throttle buttons
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);

  // quickturn
  quickturn_ = wheel_.MakeButton(5);

  // vision buttons
  exit_auto_ = throttle_.MakeButton(6);
  test_auto_ = throttle_.MakeButton(8);
  vision_intake_ = throttle_.MakeButton(2);
  drive_straight_ = throttle_.MakeButton(7);
  vision_ = throttle_.MakeButton(1);
}

void TeleopBase::operator()() {
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(20));
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

  WebdashProto webdash_proto;

  auto_status_reader_.ReadLastMessage(&auto_status);

  SuperstructureStatusProto superstructure_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &superstructure_status);

  has_cargo_ = superstructure_status->has_cargo();
  has_hp_hatch_ = superstructure_status->has_hp_hatch();
  has_ground_hatch_ = superstructure_status->has_ground_hatch();

  nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
  std::shared_ptr<nt::NetworkTable> table = inst.GetTable("limelight-front");
  std::shared_ptr<nt::NetworkTable> back_table =
      inst.GetTable("limelight-back");
  std::shared_ptr<nt::NetworkTable> expensive_table =
      inst.GetTable("limelight-pricey");

  if (RobotController::IsSysActive()) {
    if (DriverStation::GetInstance().IsOperatorControl() ||
        !auto_status->running_command()) {
      SendDrivetrainMessage();
      SendSuperstructureMessage();
    }
    if (climb_mode_) {
      table->PutNumber("ledMode", 0);
      expensive_table->PutNumber("ledMode", 0);
      back_table->PutNumber("ledMode", 0);
    } else if (superstructure_status->wrist_goal() < (M_PI / 2.)) {
      table->PutNumber(
          "ledMode",
          static_cast<int>(superstructure_status->elevator_goal() > 0.6));
      expensive_table->PutNumber(
          "ledMode",
          static_cast<int>(superstructure_status->elevator_goal() < 0.6));
      back_table->PutNumber("ledMode", flash_ ? 2 : 1);
    } else {
      table->PutNumber("ledMode", flash_ ? 2 : 1);
      expensive_table->PutNumber("ledMode", 1);
      back_table->PutNumber("ledMode", 0);
    }
  } else {
    table->PutNumber("ledMode", flash_ ? 2 : 1);
    back_table->PutNumber("ledMode", flash_ ? 2 : 1);
    expensive_table->PutNumber("ledMode", 1);
  }

  expensive_table->PutNumber("stream", 2);

  if ((has_cargo_ && !had_cargo_) || (has_hp_hatch_ && !had_hp_hatch_) ||
      (has_ground_hatch_ && !had_ground_hatch_)) {
    rumble_ticks_left_ = kRumbleTicks;
  }

  if ((has_cargo_ && !had_cargo_) || (has_hp_hatch_ && !had_hp_hatch_)) {
    flash_ticks_left_ = 0;
  }

  if (flash_ticks_left_ < 50) {
    flash_ = true;
    flash_ticks_left_++;
  } else {
    flash_ = false;
  }

  had_cargo_ = has_cargo_;
  had_hp_hatch_ = has_hp_hatch_;
  had_ground_hatch_ = has_ground_hatch_;

  if (rumble_ticks_left_ > 0) {
    if (!has_cargo_ && !has_hp_hatch_ && !has_ground_hatch_) {
      rumble_ticks_left_ = 0;
    }
    // Set rumble on
    rumble_ticks_left_--;
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 1.0);
  } else {
    // Set rumble off
    gamepad_.wpilib_joystick()->SetRumble(GenericHID::kLeftRumble, 0.0);
  }

  if (exit_auto_->was_clicked()) {
    cancel_command_ = true;
  }

  auto_goal->set_cancel_command(cancel_command_);
  auto_goal_queue_->WriteMessage(auto_goal);

  ds_sender_.Send();

  // Camera "logic"

  std::string url = "";

  if (superstructure_status->wrist_goal() > 1.57) {
    url = "10.16.78.13:5800";
  } else {
    url = "10.16.78.13:5800";
  }

  webdash_proto->set_stream_url(url);
  webdash_queue_->WriteMessage(webdash_proto);
}

void TeleopBase::SendDrivetrainMessage() {
  bool vision = false;
  double y_int = 0;
  DrivetrainGoal drivetrain_goal;
  LimelightStatusProto lime_status;
  SuperstructureStatusProto super_status;
  DrivetrainStatus drivetrain_status;
  QueueManager<SuperstructureStatusProto>::Fetch()->ReadLastMessage(
      &super_status);
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drivetrain_status);

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
  if (QueueManager<LimelightStatusProto>::Fetch()->ReadLastMessage(
          &lime_status)) {
    if (vision_->is_pressed()) {
      if (vision_intake_->is_pressed() && lime_status->back_has_target() &&
          lime_status->back_limelight_ok()) {
        vision = false;
        distance_factor_ = -0.8;
        target_dist_ = lime_status->back_target_dist();
        horiz_angle_ = lime_status->back_horiz_angle();
        y_int = -0;
      } else if (!vision_intake_->is_pressed()) {
        distance_factor_ = 1;
        y_int = 0.4;
        if (super_status->elevator_height() > 0.8) {
          horiz_angle_ = lime_status->pricey_horiz_angle();
          target_dist_ = lime_status->pricey_target_dist();
          distance_factor_ = 3.0 / 4.5;
          y_int = -0.1;
          vision = lime_status->bottom_limelight_ok() &&
                   lime_status->pricey_has_target();
        } else if (super_status->wrist_angle() > 1.5) {
          horiz_angle_ = lime_status->back_horiz_angle();
          target_dist_ = lime_status->back_target_dist();
          distance_factor_ = -4.0 / 4.5;
          y_int = 0.25;
          vision = lime_status->back_limelight_ok() &&
                   lime_status->back_has_target();
        } else {
          horiz_angle_ = lime_status->horiz_angle();
          target_dist_ = lime_status->target_dist();
          vision = lime_status->limelight_ok() && lime_status->has_target();
          double skew = lime_status->skew();
          if (lime_status->skew() > -45) {
            skew = std::abs(lime_status->skew());
          } else {
            skew += 90;
          }

          if (skew > 5 || this_run_off_) {
            horiz_angle_ += offset_;
            if (!this_run_off_) {
              offset_ =
                  0.05 * (lime_status->to_the_left() ? 1 : -1) * (skew / 20);
            }
            this_run_off_ = true;
          }
        }
      }
    }
  }

  if (vision_->was_released()) {
    this_run_off_ = false;
    offset_ = 0.;
  }

  if (super_status->elevator_height() < 1.3 &&
      super_status->elevator_goal() > 1.5) {
    vision = false;
  }

  drivetrain_goal->set_high_gear(high_gear_);

  if (!vision) {
    drivetrain_goal->mutable_teleop_goal()->set_steering(-wheel);
    drivetrain_goal->mutable_teleop_goal()->set_throttle(throttle);
    drivetrain_goal->mutable_teleop_goal()->set_quick_turn(quickturn);
  } else {
    drivetrain_goal->mutable_arc_goal()->set_angular(horiz_angle_);
    double voltage = std::abs((target_dist_ - y_int) * distance_factor_ * 4.5);
    double scalar = 1.5;
    if (drivetrain_status->linear_velocity() > 2.0) {
      scalar = 1.0;
    }
    voltage = muan::utils::Cap(voltage * scalar - (std::abs(horiz_angle_) * 20),
                               1.6, 12);
    voltage = std::copysign(voltage, distance_factor_);
    drivetrain_goal->mutable_arc_goal()->set_linear(voltage);
  }

  QueueManager<DrivetrainGoal>::Fetch()->WriteMessage(drivetrain_goal);
}

void TeleopBase::SendSuperstructureMessage() {
  SuperstructureGoalProto superstructure_goal;

  superstructure_goal->set_score_goal(c2019::superstructure::NONE);
  superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_NONE);

  double godmode_elevator = -gamepad_.wpilib_joystick()->GetRawAxis(5);
  double godmode_wrist = gamepad_.wpilib_joystick()->GetRawAxis(4);

  if (std::abs(godmode_elevator) > kGodmodeButtonThreshold) {
    superstructure_goal->set_elevator_god_mode_goal(
        (std::pow((std::abs(godmode_elevator) - kGodmodeButtonThreshold), 2) *
         kGodmodeElevatorMultiplier * (godmode_elevator > 0 ? 1 : -1)));
  }
  if (std::abs(godmode_wrist) > kGodmodeButtonThreshold) {
    superstructure_goal->set_wrist_god_mode_goal(
        (std::pow((std::abs(godmode_wrist) - kGodmodeButtonThreshold), 2) *
         kGodmodeWristMultiplier * (godmode_wrist > 0 ? 1 : -1)));
  }

  // Intake elevator height
  if (ground_intake_height_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::GROUND);
  }

  // Intake buttons
  if (cargo_intake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_CARGO);
  } else if (cargo_outtake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::OUTTAKE_CARGO);
  } else if (hp_hatch_intake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_HATCH);
  } else if (hp_hatch_outtake_->is_pressed()) {
    superstructure_goal->set_intake_goal(c2019::superstructure::OUTTAKE_HATCH);
  } else {
    superstructure_goal->set_intake_goal(c2019::superstructure::INTAKE_NONE);
  }

  // Scoring positions - auto detects game piece
  if (stow_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::STOW);
  }
  if (level_1_->is_pressed()) {
    if (!climb_mode_) {
      if (has_cargo_) {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_ROCKET_FIRST);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_ROCKET_BACKWARDS);
        }
      } else {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_ROCKET_FIRST);
          if (has_hp_hatch_) {
            superstructure_goal->set_score_goal(c2019::superstructure::GROUND);
          }
          if (has_hp_hatch_) {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::PREP_SCORE);
          } else {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::INTAKE_HATCH);
          }
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_ROCKET_BACKWARDS);
          if (has_hp_hatch_) {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::PREP_SCORE);
          } else {
            superstructure_goal->set_intake_goal(
                c2019::superstructure::INTAKE_HATCH);
          }
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::DROP_CRAWLERS);
    }
  }
  if (level_2_->is_pressed()) {
    if (!climb_mode_) {
      if (has_cargo_) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_SECOND);
      } else {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_SECOND);
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::KISS);
    }
  }
  if (level_3_->is_pressed()) {
    if (!climb_mode_) {
      if (has_cargo_) {
        superstructure_goal->set_score_goal(
            c2019::superstructure::CARGO_ROCKET_THIRD);
      } else {
        superstructure_goal->set_score_goal(
            c2019::superstructure::HATCH_ROCKET_THIRD);
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::BUST_DOWN);
    }
  }
  if (ship_->is_pressed()) {
    if (!climb_mode_) {
      if (has_cargo_) {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_SHIP_FORWARDS);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::CARGO_SHIP_BACKWARDS);
        }
      } else {
        if (!backwards_->is_pressed()) {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_SHIP_FORWARDS);
        } else {
          superstructure_goal->set_score_goal(
              c2019::superstructure::HATCH_SHIP_BACKWARDS);
        }
        if (has_hp_hatch_) {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::PREP_SCORE);
        } else {
          superstructure_goal->set_intake_goal(
              c2019::superstructure::INTAKE_HATCH);
        }
      }
    } else {
      superstructure_goal->set_score_goal(c2019::superstructure::CLIMB);
    }
  }

  // rezero mode
  if (rezero_->is_pressed() && drop_forks_->is_pressed()) {
    superstructure_goal->set_score_goal(c2019::superstructure::REZERO);
  }

  // Climbing buttons
  if (drop_forks_->is_pressed() && climb_mode_) {
    superstructure_goal->set_score_goal(c2019::superstructure::DROP_FORKS);
  }
  if (winch_left_->is_pressed()) {
    superstructure_goal->set_manual_left_winch(true);
  }
  if (winch_right_->is_pressed()) {
    superstructure_goal->set_manual_right_winch(true);
  }

  if (hp_hatch_intake_->is_pressed() && hp_hatch_outtake_->is_pressed() &&
      cargo_intake_->is_pressed() && cargo_outtake_->is_pressed()) {
    climb_mode_ = true;
  }

  if (climb_mode_ && ground_intake_height_->is_pressed()) {
    climb_mode_ = false;
  }

  superstructure_goal->set_climb_mode(climb_mode_);

  superstructure_goal_queue_->WriteMessage(superstructure_goal);
}

}  // namespace teleop
}  // namespace c2019
