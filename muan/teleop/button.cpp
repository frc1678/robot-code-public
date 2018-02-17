#include "muan/teleop/button.h"
#include "muan/teleop/joystick.h"

namespace muan {

namespace teleop {

Button::Button(Joystick* joystick, uint32_t button)
    : joystick_{joystick}, id_{button} {}

bool Button::was_clicked() { return current_ && !last_; }

bool Button::was_released() { return !current_ && last_; }

bool Button::is_pressed() { return current_; }

void Button::Update() {
  Update(joystick_->wpilib_joystick()->GetRawButton(id_));
}

void Button::Update(bool value) {
  last_ = current_;
  current_ = value;
}

PovButton::PovButton(Joystick* joystick, uint32_t button, Pov position)
    : Button(joystick, button), pov_position_(position) {}

void PovButton::Update() {
  Button::Update(joystick_->wpilib_joystick()->GetPOV(id_) ==
                 static_cast<int>(pov_position_));
}

PovRange::PovRange(Joystick* joystick, uint32_t button, int minimum,
                   int maximum)
    : Button(joystick, button), minimum(minimum), maximum(maximum) {}

void PovRange::Update() {
  int pov_in_degrees = joystick_->wpilib_joystick()->GetPOV(id_);
  bool pov_in_range = (pov_in_degrees > minimum && pov_in_degrees < maximum);
  Button::Update(pov_in_range);
}

AxisButton::AxisButton(Joystick* joystick, uint32_t button,
                       double trigger_threshold)
    : Button(joystick, button) {
  trigger_threshold_ = trigger_threshold;
}

void AxisButton::Update() {
  Button::Update(muan::utils::signum(joystick_->wpilib_joystick()->GetRawAxis(
                     id_)) == muan::utils::signum(trigger_threshold_) &&
                 std::abs(joystick_->wpilib_joystick()->GetRawAxis(id_)) >=
                     trigger_threshold_);
}

}  // namespace teleop

}  // namespace muan
