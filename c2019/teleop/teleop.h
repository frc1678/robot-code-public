#ifndef C2019_TELEOP_TELEOP_H_
#define C2019_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "c2019/commands/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/teleop/joystick.h"
#include "muan/utils/threading_utils.h"
#include "muan/wpilib/ds_sender.h"

namespace c2019 {
namespace teleop {

class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  void Update();
  void SendDrivetrainMessage();
  void SendArmMessage();

  muan::wpilib::DriverStationSender ds_sender_;
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  commands::AutoStatusQueue::QueueReader auto_status_reader_;
  commands::AutoGoalQueue *auto_goal_queue_;

  muan::teleop::Button *shifting_high_, *shifting_low_, *quickturn_,
      *exit_auto_;
  muan::teleop::Button *test_auto_, *drive_straight_;

  bool high_gear_;
  bool running_command_;
};

}  // namespace teleop
}  // namespace c2019

#endif  // C2019_TELEOP_TELEOP_H_
