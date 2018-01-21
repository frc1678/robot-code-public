#ifndef C2018_CITRUS_ROBOT_MAIN_H_
#define C2018_CITRUS_ROBOT_MAIN_H_

#include <atomic>
#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"

namespace c2018 {
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
  void SendSuperstructureMessage();
  void SendDrivetrainMessage();

  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  muan::wpilib::DriverStationSender ds_sender_;
};

}  // namespace citrus_robot
}  // namespace c2018

#endif  // C2018_CITRUS_ROBOT_MAIN_H_
