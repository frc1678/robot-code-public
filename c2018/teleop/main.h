#ifndef C2018_TELEOP_MAIN_H_
#define C2018_TELEOP_MAIN_H_

#include <atomic>
#include "WPILib.h"
#include "c2018/subsystems/climber/queue_types.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

namespace c2018 {
namespace teleop {

class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  // Runs at ~200hz
  void Update();

  void SetReadableLogName();

  void SendDrivetrainMessage();
  void SendScoreSubsystemMessage();
  void SendClimbSubsystemMessage();

  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button *quickturn_;

  // Gamepad Buttons
  muan::teleop::Button *outtake_, *intake_, *score_back_, *score_front_;
  muan::teleop::Button *initialize_climb_, *climb_, *stop_climb_, *godmode_;
  // Gamepad POVs
  muan::teleop::Button *height_0_, *height_1_, *height_2_;
  // Gamepad Axes
  muan::teleop::Button *godmode_up_, *godmode_down_, *top_mode_, *bottom_mode_;

  bool god_mode_ = false;

  muan::wpilib::DriverStationSender ds_sender_;

  bool log_name_set_ = false;

  c2018::climber::ClimberGoalProto climber_goal_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_subsystem_goal_;

  c2018::climber::ClimberGoalQueue *climber_goal_queue_;
  c2018::score_subsystem::ScoreSubsystemGoalQueue *score_subsystem_goal_queue_;
};

}  // namespace teleop
}  // namespace c2018

#endif  // C2018_TELEOP_MAIN_H_
