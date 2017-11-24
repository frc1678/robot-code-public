#ifndef O2017_WPILIB_WPILIB_INTERFACE_H_
#define O2017_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/units/units.h"
#include "o2017/queue_manager/queue_manager.h"
#include "o2017/wpilib/drivetrain_interface.h"
#include "o2017/wpilib/superstructure_interface.h"

namespace o2017 {
namespace wpilib {

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
  muan::wpilib::gyro::GyroReader gyro_;
  DrivetrainInterface drivetrain_interface_;
  SuperstructureInterface superstructure_interface_;
};

}  // namespace wpilib

}  // namespace o2017

#endif  // O2017_WPILIB_WPILIB_INTERFACE_H_
