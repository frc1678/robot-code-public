#include "o2018/teleop/teleop.h"
#include "muan/logging/logger.h"

namespace o2018 {
namespace teleop {

using muan::queues::QueueManager;
using muan::teleop::JoystickStatusProto;
using muan::wpilib::DriverStationProto;
using muan::wpilib::GameSpecificStringProto;
using o2018::subsystems::arm::IntakeMode;
using ArmGoal = o2018::subsystems::arm::ArmGoalProto;
using ArmStatus = o2018::subsystems::arm::ArmStatusProto;
using DrivetrainGoal = muan::subsystems::drivetrain::GoalProto;

TeleopBase::TeleopBase()
    : ds_sender_{QueueManager<DriverStationProto>::Fetch(),
                 QueueManager<GameSpecificStringProto>::Fetch()},
      throttle_{1, QueueManager<JoystickStatusProto>::Fetch("throttle")},
      wheel_{0, QueueManager<JoystickStatusProto>::Fetch("wheel")},
      gamepad_{2, QueueManager<JoystickStatusProto>::Fetch("gamepad")} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = wheel_.MakeButton(5);

  pos_0_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::A_BUTTON));
  pos_1_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::B_BUTTON));
  pos_2_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::X_BUTTON));
  pos_3_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::Y_BUTTON));
  // Various intake type buttons
  intake_ = gamepad_.MakeAxis(3, 0.3);
  settle_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_CLICK_IN));
  intake_open_ = gamepad_.MakeButton(uint32_t(muan::teleop::XBox::LEFT_BUMPER));
  intake_close_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_BUMPER));

  // Outtake buttons
  outtake_slow_ = gamepad_.MakeAxis(2, 0.7);
  outtake_fast_ =
      gamepad_.MakeButton(uint32_t(muan::teleop::XBox::RIGHT_CLICK_IN));
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
    SendArmMessage();
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

void TeleopBase::SendArmMessage() {
  ArmGoal arm_goal;

  double godmode_wrist = -gamepad_.wpilib_joystick()->GetRawAxis(5);
  double arm_godmode = 0;

  if (std::abs(godmode_wrist) > kGodmodeButtonThreshold) {
    arm_godmode =
        (std::pow((std::abs(godmode_wrist) - kGodmodeButtonThreshold), 2) *
         kGodmodeWristMultiplier * (godmode_wrist > 0 ? 1 : -1));
  }

  arm_goal->set_arm_god_mode_goal(0);

  if (pos_0_->is_pressed()) {
    arm_angle_ = 0;
  } else if (pos_1_->is_pressed()) {
    arm_angle_ = 20 * (M_PI / 180);
  } else if (pos_2_->is_pressed()) {
    arm_angle_ = 45 * (M_PI / 180);
  } else if (pos_3_->is_pressed()) {
    arm_angle_ = 60 * (M_PI / 180);
  }

  arm_angle_ += arm_godmode * (M_PI / 180.);
  if (arm_angle_ < 0) {
    arm_angle_ = 0;
  }
  if (arm_angle_ > M_PI / 2.) {
    arm_angle_ = M_PI / 2.;
  }
  arm_goal->set_arm_angle(arm_angle_);

  if (intake_->is_pressed()) {
    arm_goal->set_intake_mode(IntakeMode::INTAKE);
  } else if (intake_open_->is_pressed()) {
    ArmStatus arm_status;
    if (arm_status_reader_.ReadLastMessage(&arm_status) &&
        arm_status->has_cube()) {
      arm_goal->set_intake_mode(IntakeMode::DROP);
    } else {
      arm_goal->set_intake_mode(IntakeMode::INTAKE_OPEN);
    }
  } else if (intake_close_->is_pressed()) {
    arm_goal->set_intake_mode(IntakeMode::INTAKE_CLOSE);
  } else if (outtake_fast_->is_pressed()) {
    arm_goal->set_intake_mode(IntakeMode::OUTTAKE_FAST);
  } else if (outtake_slow_->is_pressed()) {
    arm_goal->set_intake_mode(IntakeMode::OUTTAKE_SLOW);
  } else if (settle_->is_pressed()) {
    arm_goal->set_intake_mode(IntakeMode::SETTLE);
  } else {
    arm_goal->set_intake_mode(IntakeMode::INTAKE_NONE);
  }

  QueueManager<ArmGoal>::Fetch()->WriteMessage(arm_goal);
}

}  // namespace teleop
}  // namespace o2018
