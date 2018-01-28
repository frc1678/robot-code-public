#ifndef C2018_TELEOP_MAIN_H_
#define C2018_TELEOP_MAIN_H_

#include <atomic>
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"

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
  muan::teleop::Button* elevator_up_;
  c2018::score_subsystem::ScoreSubsystemGoalProto elevator_goal;

  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_goal_queue_ = QueueManager<ScoreSubsystemGoalProto>::Fetch();

  muan::wpilib::DriverStationSender ds_sender_;

  bool log_name_set_ = false;
};

}  // namespace teleop
}  // namespace c2018

#endif  // C2018_TELEOP_MAIN_H_
