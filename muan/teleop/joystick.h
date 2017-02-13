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

enum class JoystickType { WHEEL = 0, THROTTLE = 1, XBOX = 2 };

class Joystick {
 public:
  explicit Joystick(int32_t port, JoystickType type = JoystickType::WHEEL);
  explicit Joystick(int32_t port, JoystickStatusQueue* queue, JoystickType type = JoystickType::WHEEL);

  void Update();

  muan::teleop::Button* MakeButton(uint32_t button);
  muan::teleop::Button* MakePov(uint32_t pov, Pov position);
  muan::teleop::Button* MakeAxis(uint32_t button);

  ::Joystick* wpilib_joystick();

 private:
  void LogButtons();
  uint32_t get_button_count();

  JoystickStatusQueue* queue_;
  JoystickType type_;

  std::vector<std::unique_ptr<Button>> buttons_;
  ::Joystick wpilib_joystick_;
};

}  // namespace teleop

}  // namespace muan

#endif  // MUAN_TELEOP_JOYSTICK_H_
