#ifndef C2017_TELEOP_TELEOP_H_
#define C2017_TELEOP_TELEOP_H_

#include "muan/teleop/joystick.h"

namespace c2017 {

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
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  void SendDSMessage();
};

}  // wpilib_update

}  // c2017

#endif  // C2017_TELEOP_TELEOP_H_
