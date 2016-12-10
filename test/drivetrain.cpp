#include “drivetrain.h”

DriveTrain::Update(double wheel, double joystick, bool quickturn){
  if quickturn {
    double left = wheel * 12;
    double right = -(wheel * 12);
    return std::make_tuple(left, right);
  }
  else {
    double left = joystick * 6 + wheel * 6 * joystick;
    double right = joystick * 6 – wheel * 6 * joystick;
    return std::make_tuple(left, right);
  }
}
