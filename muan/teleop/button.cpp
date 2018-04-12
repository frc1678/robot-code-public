#include "muan/teleop/button.h"
#include <cmath>
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

PovRange::PovRange(Joystick* joystick, uint32_t button, double minimum,
                   double maximum)
    : Button(joystick, button), minimum_(minimum), maximum_(maximum) {}

void PovRange::Update() {
  int pov_in_degrees = joystick_->wpilib_joystick()->GetPOV(id_);
  bool pov_in_range = (pov_in_degrees > minimum_ && pov_in_degrees < maximum_);
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

AxisRange::AxisRange(Joystick* joystick, double minimum, double maximum,
                     double xaxis, double yaxis, double threshold)
    : Button(joystick, xaxis),
      minimum_(minimum),
      maximum_(maximum),
      yaxis_(yaxis),
      threshold_(threshold) {}

void AxisRange::Update() {
  double xaxis = joystick_->wpilib_joystick()->GetRawAxis(id_);
  double yaxis = joystick_->wpilib_joystick()->GetRawAxis(yaxis_);
  double axis_in_degrees = (atan2(yaxis, xaxis)) * (180 / M_PI);
  if (axis_in_degrees > -90 && axis_in_degrees < 180) {  // conform to wpilib
    axis_in_degrees += 90;
  } else {
    axis_in_degrees += 450;
  }
  bool axis_in_range =
      (axis_in_degrees > minimum_ && axis_in_degrees < maximum_);
  bool past_threshold =
      (xaxis * xaxis) + (yaxis * yaxis) > (threshold_ * threshold_);
  Button::Update(axis_in_range && past_threshold);
}

}  // namespace teleop

}  // namespace muan
