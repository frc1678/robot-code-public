#include "muan/teleop/joystick.h"
#include "muan/teleop/button.h"

namespace muan {

namespace teleop {

Joystick::Joystick(int32_t port) : wpilib_joystick_{port} {
  queue_ = nullptr;
  for (uint32_t button_id = 1; button_id <= uint32_t(wpilib_joystick_.GetButtonCount()); button_id++) {
    buttons_.emplace_back(new Button(this, button_id));
  }
}

Joystick::Joystick(int32_t port, JoystickStatusQueue* queue) : wpilib_joystick_{port} {
  queue_ = queue;
  for (uint32_t button_id = 1; button_id <= uint32_t(wpilib_joystick_.GetButtonCount()); button_id++) {
    buttons_.emplace_back(new Button(this, button_id));
  }
}

::Joystick* Joystick::wpilib_joystick() { return &wpilib_joystick_; }

Button* Joystick::MakeButton(uint32_t button_id) { return buttons_[button_id - 1].get(); }

muan::teleop::Button* Joystick::MakePov(uint32_t button, Pov position) {
  buttons_.emplace_back(new muan::teleop::PovButton(this, button, position));
  return buttons_[buttons_.size() - 1].get();
}

muan::teleop::Button* Joystick::MakeAxis(uint32_t button) {
  buttons_.emplace_back(new muan::teleop::AxisButton(this, button, .7));
  return buttons_[buttons_.size() - 1].get();
}

void Joystick::Update() {
  for (auto& button : buttons_) {
    button->Update();
  }
  if (queue_) {
    LogButtons();
  }
}

void Joystick::LogButtons() {
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
  if (buttons_.size() > 10) {
    joystick_status->set_button_eleven(buttons_[10].get()->is_pressed());
    if (buttons_.size() > 11) {
      joystick_status->set_button_twelve(buttons_[11].get()->is_pressed());
      joystick_status->set_button_thirteen(buttons_[12].get()->is_pressed());
    } else {
      joystick_status->set_button_twelve(0);
      joystick_status->set_button_thirteen(0);
    }
  } else {
    joystick_status->set_button_eleven(0);
    joystick_status->set_button_twelve(0);
    joystick_status->set_button_thirteen(0);
  }

  joystick_status->set_axis_one(wpilib_joystick_.GetRawAxis(0));
  joystick_status->set_axis_two(wpilib_joystick_.GetRawAxis(1));
  joystick_status->set_axis_three(wpilib_joystick_.GetRawAxis(2));
  if (buttons_.size() < 13) {
    joystick_status->set_axis_four(wpilib_joystick_.GetRawAxis(3));
    if (buttons_.size() < 11) {
      joystick_status->set_axis_five(wpilib_joystick_.GetRawAxis(4));
      joystick_status->set_axis_six(wpilib_joystick_.GetRawAxis(5));
    } else {
      joystick_status->set_axis_five(0);
      joystick_status->set_axis_six(0);
    }
  } else {
    joystick_status->set_axis_four(0);
    joystick_status->set_axis_five(0);
    joystick_status->set_axis_six(0);
  }
  queue_->WriteMessage(joystick_status);
}

}  // namespace teleop
}  // namespace muan
