#include "muan/teleop/joystick.h"
#include "muan/teleop/button.h"

namespace muan {

namespace teleop {

Joystick::Joystick(int32_t port) : wpilib_joystick_{port} {}
Joystick::Joystick(int32_t port, JoystickStatusQueue* queue) : wpilib_joystick_{port} { queue_ = queue; }

::Joystick* Joystick::wpilib_joystick() { return &wpilib_joystick_; }

Button* Joystick::MakeButton(uint32_t button) {
  buttons_.emplace_back(new Button(this, button));
  return buttons_[buttons_.size() - 1].get();
}

void Joystick::Update() {
  for (auto& button : buttons_) {
    button->Update();
  }
  LogButtons(0);
}

void Joystick::LogButtons(uint32_t dummy) {
  dummy = dummy * 2;  // Doing this because it warns of unused parameters

  JoystickStatusProto joystick_status;

  joystick_status->set_button_one(buttons_[0].get()->is_pressed());
  joystick_status->set_button_two(buttons_[1].get()->is_pressed());
  joystick_status->set_button_three(buttons_[2].get()->is_pressed());
  joystick_status->set_button_four(buttons_[3].get()->is_pressed());
  joystick_status->set_button_five(buttons_[4].get()->is_pressed());
  joystick_status->set_button_six(buttons_[5].get()->is_pressed());
  joystick_status->set_button_seven(buttons_[6].get()->is_pressed());
  joystick_status->set_button_eight(buttons_[7].get()->is_pressed());
  joystick_status->set_button_nine(buttons_[8].get()->is_pressed());
  joystick_status->set_button_ten(buttons_[9].get()->is_pressed());
  joystick_status->set_button_eleven(buttons_[10].get()->is_pressed());
  joystick_status->set_button_twelve(buttons_[11].get()->is_pressed());

  joystick_status->set_axis_one(wpilib_joystick_.GetRawAxis(0));
  joystick_status->set_axis_two(wpilib_joystick_.GetRawAxis(1));
  joystick_status->set_axis_three(wpilib_joystick_.GetRawAxis(2));
  joystick_status->set_axis_four(wpilib_joystick_.GetRawAxis(3));

  queue_->WriteMessage(joystick_status);
}

void Joystick::LogButtons(int dummy) { dummy = dummy * 2; }
}  // namespace teleop

}  // namespace muan
