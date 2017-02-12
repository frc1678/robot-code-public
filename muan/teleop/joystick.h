#ifndef MUAN_TELEOP_JOYSTICK_H_
#define MUAN_TELEOP_JOYSTICK_H_

#include <cstdint>
#include <memory>
#include <vector>
#include "WPILib.h"
#include "muan/teleop/button.h"
#include "muan/teleop/queue_types.h"

namespace muan {

namespace teleop {

class Joystick {
 public:
  explicit Joystick(int32_t port);
  explicit Joystick(int32_t port, JoystickStatusQueue* queue);

  void Update();

  Button* MakeButton(uint32_t button);

  ::Joystick* wpilib_joystick();

 private:
  void LogButtons(uint32_t dummy);
  void LogButtons(int dummy);


  JoystickStatusQueue* queue_ = nullptr;

  std::vector<std::unique_ptr<Button>> buttons_;
  ::Joystick wpilib_joystick_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_JOYSTICK_H_
