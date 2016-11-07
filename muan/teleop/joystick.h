#ifndef MUAN_TELEOP_JOYSTICK_H_
#define MUAN_TELEOP_JOYSTICK_H_

#include "WPILib.h"
#include "button.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace muan {

namespace teleop {

class Joystick {
 public:
  Joystick(uint32_t port);
  void Update();

  muan::teleop::Button* MakeButton(uint32_t button);

  ::Joystick* wpilib_joystick();

 private:
  std::vector<std::unique_ptr<muan::teleop::Button>> buttons_;
  ::Joystick wpilib_joystick_;
};

}  // teleop

}  // muan

#endif  // MUAN_TELEOP_JOYSTICK_H_
