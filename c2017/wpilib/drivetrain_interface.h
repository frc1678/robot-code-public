#ifndef C2017_WPILIB_DRIVETRAIN_INTERFACE_H_
#define C2017_WPILIB_DRIVETRAIN_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "c2017/queue_manager/queue_manager.h"
#include "muan/utils/math_utils.h"

namespace c2017 {

namespace wpilib {

class DrivetrainInterface {
 public:
  DrivetrainInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  frc971::control_loops::drivetrain::InputQueue* input_queue_;
  frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  VictorSP motor_left_;
  VictorSP motor_right_;

  Encoder encoder_left_, encoder_right_;
};
}
}

#endif
