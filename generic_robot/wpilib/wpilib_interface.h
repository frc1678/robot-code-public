#ifndef GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
#define GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"

namespace generic_robot {

namespace wpilib {

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
};

}  // namespace wpilib

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
