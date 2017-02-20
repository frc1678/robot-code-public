#include "muan/teleop/joystick.h"
#include "muan/teleop/button.h"

namespace muan {

namespace teleop {

Joystick::Joystick(int32_t port) : wpilib_joystick_{port} {
  queue_ = nullptr;
  for (uint32_t button_id = 1; button_id <= uint32_t(wpilib_joystick_.GetButtonCount()); button_id++) {
    buttons_.emplace_back(new Button(this, button_id));
  }
  for (uint32_t axis_id = 0; axis_id <= uint32_t(wpilib_joystick_.GetAxisCount()); axis_id++) {
    axis_.emplace_back(new muan::teleop::AxisButton(this, axis_id, .7));
  }
}

Joystick::Joystick(int32_t port, JoystickStatusQueue* queue) : queue_{queue}, wpilib_joystick_{port} {
  for (uint32_t button_id = 1; button_id <= uint32_t(wpilib_joystick_.GetButtonCount()); button_id++) {
    buttons_.emplace_back(new Button(this, button_id));
  }
  for (uint32_t axis_id = 1; axis_id <= uint32_t(wpilib_joystick_.GetAxisCount()); axis_id++) {
    axis_.emplace_back(new muan::teleop::AxisButton(this, axis_id, .7));
  }
}

::Joystick* Joystick::wpilib_joystick() { return &wpilib_joystick_; }

Button* Joystick::GetButton(uint32_t button_id) { return buttons_[button_id - 1].get(); }

muan::teleop::Button* Joystick::GetPov(uint32_t button, Pov position) {
  pov_.emplace_back(new muan::teleop::PovButton(this, button, position));
  return pov_[pov_.size() - 1].get();
}

muan::teleop::Button* Joystick::GetAxis(uint32_t button) {
  buttons_.emplace_back(new muan::teleop::AxisButton(this, button, .7));
  return buttons_[buttons_.size() - 1].get();
}

void Joystick::Update() {
  for (auto& button : buttons_) {
    button->Update();
  }
  for (auto& axis : axis_) {
    axis->Update();
  }
  for (auto& pov : pov_) {
    pov->Update();
  }
  if (queue_) {
    LogButtons();
  }
}

void Joystick::LogButtons() {
  JoystickStatusProto joystick_status;
  if (buttons_.size() >= 1) {
    joystick_status->set_button1(buttons_[0].get()->is_pressed());
  }
  if (buttons_.size() >= 2) {
    joystick_status->set_button2(buttons_[1].get()->is_pressed());
  }
  if (buttons_.size() >= 3) {
  joystick_status->set_button3(buttons_[2].get()->is_pressed());
  }
  if (buttons_.size() >= 4) {
  joystick_status->set_button4(buttons_[3].get()->is_pressed());
  }
  if (buttons_.size() >= 5) {
  joystick_status->set_button5(buttons_[4].get()->is_pressed());
  }
  if (buttons_.size() >= 6) {
  joystick_status->set_button6(buttons_[5].get()->is_pressed());
  }
  if (buttons_.size() >= 7) {
  joystick_status->set_button7(buttons_[6].get()->is_pressed());
  }
  if (buttons_.size() >= 8) {
  joystick_status->set_button8(buttons_[7].get()->is_pressed());
  }
  if (buttons_.size() >= 9) {
  joystick_status->set_button9(buttons_[8].get()->is_pressed());
  }
  if (buttons_.size() >= 10) {
  joystick_status->set_button10(buttons_[9].get()->is_pressed());
  }
  if (buttons_.size() > 11) {
    joystick_status->set_button11(buttons_[10].get()->is_pressed());
  }
  if (buttons_.size() > 12) {
    joystick_status->set_button12(buttons_[11].get()->is_pressed());
  }
  if (buttons_.size() >= 13) {
      joystick_status->set_button13(buttons_[12].get()->is_pressed());
  }

  if (axis_.size() >= 1) {
    joystick_status->set_axis1(wpilib_joystick_.GetRawAxis(0));
  }
  if (axis_.size() >= 2) {
    joystick_status->set_axis2(wpilib_joystick_.GetRawAxis(1));
  }
  if (axis_.size() >= 3) {
    joystick_status->set_axis3(wpilib_joystick_.GetRawAxis(2));
  }
  if (axis_.size() >= 4) {
    joystick_status->set_axis4(wpilib_joystick_.GetRawAxis(3));
  }
  if (axis_.size() >= 5) {
    joystick_status->set_axis5(wpilib_joystick_.GetRawAxis(4));
  }
  if (axis_.size() >= 6) {
    joystick_status->set_axis6(wpilib_joystick_.GetRawAxis(5));
  }
  queue_->WriteMessage(joystick_status);
}

}  // namespace teleop
}  // namespace muan
