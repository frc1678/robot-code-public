#ifndef C2018_TELEOP_MAIN_H_
#define C2018_TELEOP_MAIN_H_

#include <atomic>
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "muan/queues/queue_manager.h"
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "muan/wpilib/queue_types.h"

using muan::queues::QueueManager;

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
  void SendSuperstructureMessage();
  void SendDrivetrainMessage();

  void SetReadableLogName();

  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;
  muan::teleop::Button* start_intake_;
  muan::teleop::Button* start_outtake_;
  muan::teleop::Button* stop_intake_;
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_goal_proto_;

  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_goal_queue_ =
      QueueManager<c2018::score_subsystem::ScoreSubsystemGoalProto>::Fetch();

  muan::wpilib::DriverStationSender ds_sender_;

  bool log_name_set_ = false;
};

}  // namespace teleop
}  // namespace c2018

#endif  // C2018_TELEOP_MAIN_H_
