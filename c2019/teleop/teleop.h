#ifndef C2019_TELEOP_TELEOP_H_
#define C2019_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "c2019/commands/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/teleop/joystick.h"
#include "muan/utils/threading_utils.h"
#include "muan/wpilib/ds_sender.h"

namespace c2019 {
namespace teleop {

// TODO(Hanson) tune these with Nathan
constexpr double kGodmodeButtonThreshold = .25;
constexpr double kGodmodeElevatorMultiplier = 4;
constexpr double kGodmodeWristMultiplier = 10;

class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  c2019::superstructure::SuperstructureGoalQueue *superstructure_goal_queue_;
  c2019::superstructure::SuperstructureStatusQueue
      *superstructure_status_queue_;
  muan::webdash::WebdashQueue *webdash_queue_;
  std::atomic<bool> running_;

  void Update();
  void SendDrivetrainMessage();

  // driver controls
  muan::wpilib::DriverStationSender ds_sender_;
  muan::teleop::Joystick throttle_, wheel_;
  // operator controls
  muan::teleop::Joystick gamepad_;

  void SendSuperstructureMessage();

  // climbing buttons
  muan::teleop::Button *crawl_, *drop_forks_, *drop_crawlers_, *brake_;

  // safety button
  muan::teleop::Button *safety_, *safety2_;

  // intake/outtake buttons
  muan::teleop::Button *cargo_intake_, *cargo_outtake_, *hp_hatch_intake_,
      *hp_hatch_outtake_;
  muan::teleop::Button *ground_intake_height_;
  // scoring positions
  muan::teleop::Button *level_1_, *level_2_, *level_3_, *ship_;
  muan::teleop::Button *stow_;
  // backwards scoring mode
  muan::teleop::Button *backwards_;
  // handoff and pop goals
  muan::teleop::Button *handoff_, *pop_;
  // rezero
  muan::teleop::Button *rezero_;

  int kRumbleTicks = 25;
  int rumble_ticks_left_;

  bool ground_hatch_outtake_;

  bool has_cargo_ = false;
  bool has_hp_hatch_ = false;
  bool has_ground_hatch_ = false;
  bool had_cargo_ = false;
  bool had_hp_hatch_ = false;
  bool had_ground_hatch_ = false;

  bool climb_mode_ = false;
  int counter_ = 0;

  // vision buttons
  commands::AutoStatusQueue::QueueReader auto_status_reader_;
  commands::AutoGoalQueue *auto_goal_queue_;

  muan::teleop::Button *shifting_high_, *shifting_low_, *quickturn_,
      *exit_auto_;
  muan::teleop::Button *test_auto_, *drive_straight_, *vision_, *vision_intake_,
      *winch_, *winch_left_, *winch_right_;

  bool high_gear_;
  /* bool running_command_; */
  double distance_factor_;
  double last_horiz_angle_;
  bool wants_override_;
  double tx_error_;
  double estimated_heading_;
  double current_heading_;
  double cached_velocity_;
  double target_dist_;
  double pricey_horiz_angle_;
  double horiz_angle_;
  int velocity_counter_;
  double height_distance_factor_ = 0.0;
  superstructure::ScoreGoal override_goal_;
  superstructure::ScoreGoal cached_goal_;
  bool this_run_off_;
  int flash_ticks_left_ = 0;
  bool flash_ = false;
  double offset_ = 0;
  bool cancel_command_ = false;
};

}  // namespace teleop
}  // namespace c2019

#endif  // C2019_TELEOP_TELEOP_H_
