#ifndef O2016_TELEOP_TELEOP_H_
#define O2016_TELEOP_TELEOP_H_

#include "muan/teleop/joystick.h"

namespace o2016 {

namespace teleop {

class Teleop {
 public:
  Teleop();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  void SendDSMessage();
  void SendDrivetrainMessage();
};

}  // teleop

}  // o2016

#endif  // O2016_TELEOP_TELEOP_H_
