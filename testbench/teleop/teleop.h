#ifndef TESTBENCH_TELEOP_TELEOP_H_
#define TESTBENCH_TELEOP_TELEOP_H_

#include "muan/teleop/joystick.h"

namespace testbench {

namespace teleop {

class Teleop {
 public:
  Teleop();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;

  bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;
  muan::teleop::Button* drive_profile_;

  void SendDSMessage();
  void SendDrivetrainMessage();
};

}  // teleop

}  // testbench

#endif  // TESTBENCH_TELEOP_TELEOP_H_
