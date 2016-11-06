#include "joystick.h"
#include "button.h"

namespace muan {

namespace teleop {

Joystick::Joystick(uint32_t port) : wpilib_joystick_{port} {}

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

}  // teleop

}  // muan
