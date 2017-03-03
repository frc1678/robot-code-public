#ifndef C2017_CITRUS_ROBOT_MAIN_H_
#define C2017_CITRUS_ROBOT_MAIN_H_

#include "muan/teleop/joystick.h"
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
  muan::teleop::Button *fender_align_shoot_, *score_hp_gear_, *driver_score_ground_gear_;
  // Gamepad buttons
  muan::teleop::Button *ball_intake_toggle_, *gear_intake_down_, *operator_score_ground_gear_, *ball_reverse_,
      *just_shoot_, *climb_, *just_spinup_, *stop_shooting_, *toggle_distance_align_;
  // Gamepad D-Pad
  muan::teleop::Button *hp_load_gears_, *hp_load_balls_, *hp_load_both_;
  // Gamepad Triggers
  muan::teleop::Button *agitate_, *ball_intake_run_;

  bool ball_intake_down_ = false;
  bool currently_climbing_ = false;
  bool using_vision_ = false;
  bool use_distance_align_ = true;

  c2017::shooter_group::ShooterGroupGoalProto shooter_group_goal_;
  c2017::intake_group::IntakeGroupGoalProto intake_group_goal_;

  void SendDSMessage();
  void SendSuperstructureMessage();
  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace c2017

#endif  // C2017_CITRUS_ROBOT_MAIN_H_
