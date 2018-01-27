#ifndef C2017_CITRUS_ROBOT_CITRUS_ROBOT_H_
#define C2017_CITRUS_ROBOT_CITRUS_ROBOT_H_

#include <atomic>

#include "c2017/lemonscript/lemonscript.h"
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2017 {

namespace citrus_robot {

class CitrusRobot {
 public:
  CitrusRobot();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  // Runs at ~200hz
  void Update();

  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  c2017::lemonscript::Lemonscript lemonscript_;
  std::thread lemonscript_thread_{std::ref(lemonscript_)};

  muan::teleop::Button *quickturn_;

  // Superstructure buttons
  // Throttle button
  muan::teleop::Button *align_shoot_, *driver_score_ground_gear_;
  // Gamepad buttons
  muan::teleop::Button *gear_intake_down_, *operator_score_ground_gear_,
      *ball_reverse_, *just_shoot_, *climb_, *just_spinup_, *stop_shooting_;
  // Gamepad D-Pad
  muan::teleop::Button *toggle_magazine_, *toggle_distance_align_;
  // Gamepad Triggers
  muan::teleop::Button *agitate_, *ball_intake_fast_, *ball_intake_slow_,
      *drop_balls_;

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
  muan::wpilib::DriverStationQueue::QueueReader ds_reader_;

  void SendSuperstructureMessage();
  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace c2017

#endif  // C2017_CITRUS_ROBOT_CITRUS_ROBOT_H_
