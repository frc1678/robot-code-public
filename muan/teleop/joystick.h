#ifndef MUAN_TELEOP_JOYSTICK_H_
#define MUAN_TELEOP_JOYSTICK_H_

#include <cstdint>
#include <memory>
#include <vector>
#include "WPILib.h"
#include "muan/teleop/button.h"

namespace muan {

namespace teleop {

class Joystick {
 public:
  explicit Joystick(int32_t port);
  void Update();

  muan::teleop::Button* MakeButton(uint32_t button);

  ::Joystick* wpilib_joystick();

 private:
  std::vector<std::unique_ptr<muan::teleop::Button>> buttons_;
  ::Joystick wpilib_joystick_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_JOYSTICK_H_
