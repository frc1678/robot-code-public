#ifndef GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
#define GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/units/units.h"
#include "generic_robot/queue_manager/queue_manager.h"
#include "generic_robot/wpilib/drivetrain_interface.h"

namespace generic_robot {
namespace wpilib {

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
  DrivetrainInterface drivetrain_interface_;
};

}  // namespace wpilib

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
