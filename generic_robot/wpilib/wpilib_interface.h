#ifndef GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
#define GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "gflags/gflags.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "generic_robot/queue_manager/queue_manager.h"
#include "generic_robot/wpilib/drivetrain_interface.h"
#include "generic_robot/wpilib/superstructure_interface.h"

namespace generic_robot {
namespace wpilib {

DECLARE_int32(gyro_time);

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;
  muan::wpilib::gyro::GyroReader gyro_;
  DrivetrainInterface drivetrain_;
  SuperstructureInterface superstructure_;
};

}  // namespace wpilib
}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_WPILIB_INTERFACE_H_
