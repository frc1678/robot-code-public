#ifndef C2017_WPILIB_WPILIB_INTERFACE_H_
#define C2017_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"

namespace c2017 {

namespace wpilib {

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
};

}  // wpilib

}  // c2017

#endif  // C2017_WPILIB_WPILIB_INTERFACE_H_
