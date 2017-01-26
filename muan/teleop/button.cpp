#include "muan/teleop/button.h"
#include "muan/teleop/joystick.h"

namespace muan {

namespace teleop {

Button::Button(Joystick* joystick, uint32_t button) : joystick_{joystick}, id_{button} {}

bool Button::was_clicked() { return current_ && !last_; }

bool Button::was_released() { return !current_ && last_; }

bool Button::is_pressed() { return current_; }

void Button::Update() { Update(joystick_->wpilib_joystick()->GetRawButton(id_)); }

void Button::Update(bool value) {
  last_ = current_;
  current_ = value;
}

}  // namespace teleop

}  // namespace muan
