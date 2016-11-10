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

  muan::teleop::Button* intake_;
  muan::teleop::Button* spit_;
  muan::teleop::Button* defense_;
  muan::teleop::Button* prep_shot_;
  muan::teleop::Button* shoot_;
  muan::teleop::Button* vision_fail_toggle_;

  bool vision_fail_ = true; // I cry

  void SendDSMessage();
  void SendDrivetrainMessage();
  void SendSuperstructureMessage();
};

}  // teleop

}  // o2016

#endif  // O2016_TELEOP_TELEOP_H_
