#ifndef O2017_CITRUS_ROBOT_MAIN_H_
#define O2017_CITRUS_ROBOT_MAIN_H_

#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"

namespace o2017 {

namespace citrus_robot {

class CitrusRobot {
 public:
  CitrusRobot();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button* shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  muan::wpilib::DriverStationSender ds_sender_;

  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace o2017

#endif  // O2017_CITRUS_ROBOT_MAIN_H_
