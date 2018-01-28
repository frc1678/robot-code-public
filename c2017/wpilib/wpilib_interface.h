#ifndef C2017_WPILIB_WPILIB_INTERFACE_H_
#define C2017_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "c2017/queue_manager/queue_manager.h"
#include "gflags/gflags.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/gyro/gyro_reader.h"

#include "c2017/wpilib/drivetrain_interface.h"
#include "c2017/wpilib/lights_interface.h"
#include "c2017/wpilib/superstructure_interface.h"

namespace c2017 {
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
  SuperStructureInterface superstructure_;
  LightsInterface lights_;
};

}  // namespace wpilib
}  // namespace c2017

#endif  // C2017_WPILIB_WPILIB_INTERFACE_H_
