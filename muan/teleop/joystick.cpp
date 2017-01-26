#include "muan/teleop/joystick.h"
#include "muan/teleop/button.h"

namespace muan {

namespace teleop {

Joystick::Joystick(int32_t port) : wpilib_joystick_{port} {}

::Joystick* Joystick::wpilib_joystick() { return &wpilib_joystick_; }

muan::teleop::Button* Joystick::MakeButton(uint32_t button) {
  buttons_.emplace_back(new muan::teleop::Button(this, button));
  return buttons_[buttons_.size() - 1].get();
}

void Joystick::Update() {
  for (auto& button : buttons_) {
    button->Update();
  }
}

}  // namespace teleop

}  // namespace muan
