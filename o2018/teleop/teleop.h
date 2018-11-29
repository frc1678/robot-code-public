#ifndef O2018_TELEOP_TELEOP_H_
#define O2018_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "muan/queues/queue_manager.h"
#include "muan/subsystems/drivetrain/queue_types.h"
#include "muan/teleop/joystick.h"
#include "muan/utils/threading_utils.h"
#include "o2018/subsystems/arm/queue_types.h"
#include "muan/wpilib/ds_sender.h"

namespace o2018 {
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

  // Message Sending functions go here
  // Buttons and joysticks go here

  muan::wpilib::DriverStationSender ds_sender_;
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;
  muan::teleop::Button *shifting_high_, *shifting_low_, *quickturn_;

  muan::teleop::Button *outtake_fast_, *outtake_slow_, *intake_, *settle_,
      *intake_open_, *intake_close_;

  muan::teleop::Button *pos_0_, *pos_1_, *pos_2_, *pos_3_;

  o2018::subsystems::arm::ArmStatusQueue::QueueReader arm_status_reader_ = muan::queues::QueueManager<
                           o2018::subsystems::arm::ArmStatusProto>::Fetch()
                           ->MakeReader();

  double arm_angle_ = 0;
  bool high_gear_;

  // Godmode constants
  double kGodmodeWristMultiplier = 3;
  double kGodmodeButtonThreshold = .25;
};

}  // namespace teleop
}  // namespace o2018

#endif  // O2018_TELEOP_TELEOP_H_
