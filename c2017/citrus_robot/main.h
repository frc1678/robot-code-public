#ifndef C2017_CITRUS_ROBOT_MAIN_H_
#define C2017_CITRUS_ROBOT_MAIN_H_

#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "c2017/lemonscript/lemonscript.h"

namespace c2017 {

namespace citrus_robot {

class CitrusRobot {
 public:
  CitrusRobot();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  c2017::lemonscript::Lemonscript lemonscript_;
  std::thread lemonscript_thread_{std::ref(lemonscript_)};

  muan::teleop::Button *quickturn_;

  // Superstructure buttons
  // Throttle button
  muan::teleop::Button *align_shoot_, *driver_score_ground_gear_;
  // Gamepad buttons
  muan::teleop::Button *ball_intake_toggle_, *gear_intake_down_, *operator_score_ground_gear_, *ball_reverse_,
      *just_shoot_, *climb_, *just_spinup_, *stop_shooting_;
  // Gamepad D-Pad
  muan::teleop::Button *toggle_magazine_, *toggle_distance_align_;
  // Gamepad Triggers
  muan::teleop::Button *agitate_, *ball_intake_run_, *drop_balls_;

  bool ball_intake_down_ = false;
  bool currently_climbing_ = false;
  bool using_vision_ = false;
  bool vision_aligned_ = false;

  bool magazine_out_ = true;

  bool use_distance_align_ = false;

  c2017::shooter_group::ShooterGroupGoalProto shooter_group_goal_;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_;
  c2017::magazine::MagazineGoalProto magazine_goal_;

  muan::wpilib::DriverStationSender ds_sender_;

  void SendSuperstructureMessage();
  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace c2017

#endif  // C2017_CITRUS_ROBOT_MAIN_H_
