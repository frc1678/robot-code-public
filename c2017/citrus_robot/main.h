#ifndef C2017_CITRUS_ROBOT_MAIN_H_
#define C2017_CITRUS_ROBOT_MAIN_H_

#include "muan/teleop/joystick.h"
#include "c2017/lemonscript/lemonscript.h"

namespace c2017 {

namespace citrus_robot {

class CitrusRobot {
 public:
  CitrusRobot();

  // Call this to update at ~50hz (DS update rate)
  void Update();

 private:
  muan::teleop::Joystick throttle_, wheel_;
  muan::teleop::Joystick gamepad_;

  c2017::lemonscript::Lemonscript lemonscript_;
  std::thread lemonscript_thread_{std::ref(lemonscript_)};

  // bool high_gear_;
  muan::teleop::Button *shifting_high_, *shifting_low_;
  muan::teleop::Button* quickturn_;

  void SendDSMessage();
  void SendDrivetrainMessage();
};

}  // namespace citrus_robot

}  // namespace c2017

#endif  // C2017_CITRUS_ROBOT_MAIN_H_
