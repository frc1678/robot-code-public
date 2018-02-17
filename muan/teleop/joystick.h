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

enum class XBox {
  A_BUTTON = 1,
  B_BUTTON = 2,
  X_BUTTON = 3,
  Y_BUTTON = 4,
  LEFT_BUMPER = 5,
  RIGHT_BUMPER = 6,
  BACK = 7,
  START = 8,
  LEFT_CLICK_IN = 9,
  RIGHT_CLICK_IN = 10,
};

class Joystick {
 public:
  explicit Joystick(int32_t port);
  Joystick(int32_t port, JoystickStatusQueue* queue);
  Joystick(int32_t port, JoystickStatusQueue* queue,
           XBoxRumbleQueue* rumble_queue);

  void Update();

  muan::teleop::Button* MakeButton(uint32_t button);
  muan::teleop::Button* MakePov(uint32_t pov, Pov position);
  muan::teleop::Button* MakePovRange(uint32_t pov, int minimum, int maximum);
  muan::teleop::Button* MakeAxis(uint32_t button, double threshold);

  ::Joystick* wpilib_joystick();

 private:
  void LogButtons();

  JoystickStatusQueue* status_queue_;
  XBoxRumbleQueue* rumble_queue_;

  std::vector<std::unique_ptr<Button>> buttons_;
  ::Joystick wpilib_joystick_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_JOYSTICK_H_
