#ifndef GENERIC_ROBOT_WPILIB_UPDATE_MAIN_H_
#define GENERIC_ROBOT_WPILIB_UPDATE_MAIN_H_

#include "muan/teleop/joystick.h"
#include "muan/wpilib/ds_sender.h"

namespace generic_robot {

namespace wpilib_update {

class Main {
 public:
  Main();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button* shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  muan::wpilib::DriverStationSender ds_sender_;
};

}  // namespace wpilib_update

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_UPDATE_MAIN_H_
