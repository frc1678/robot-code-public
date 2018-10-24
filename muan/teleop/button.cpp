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

// Custom button bounded to a range defined by a minimum and maximum
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

// Custom button bounded to a range defined by a minimum and maximum
AxisRange::AxisRange(Joystick* joystick, double minimum, double maximum,
                     double x_axis, double y_axis, double threshold)
    : Button(joystick, x_axis),
      minimum_(minimum),
      maximum_(maximum),
      y_axis_(y_axis),
      threshold_(threshold) {}

void AxisRange::Update() {
  // Get x and y position of button
  double x_axis = joystick_->wpilib_joystick()->GetRawAxis(id_);
  double y_axis = joystick_->wpilib_joystick()->GetRawAxis(y_axis_);

  // Pythagorean theorem to determine button magnitude
  double magnitude = sqrt((x_axis * x_axis) + (y_axis * y_axis));

  // Simple trig to determine button angle
  double axis_in_degrees = std::abs((atan2(y_axis, x_axis)) * (180 / M_PI));

  // Conform to wpilib's angle system
  bool axis_in_range =
      (axis_in_degrees >= minimum_ && axis_in_degrees <= maximum_);
  bool past_threshold =
      (x_axis * x_axis) + (y_axis * y_axis) > (threshold_ * threshold_);

  // If button is in the angle range and above threshold then it is pressed
  // It needs (magnitude > 0.1) as a condition because sketchy Xbox controller
  Button::Update(axis_in_range && past_threshold && (magnitude > 0.1));
}

}  // namespace teleop

}  // namespace muan
