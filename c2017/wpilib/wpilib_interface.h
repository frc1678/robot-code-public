#ifndef C2017_WPILIB_WPILIB_INTERFACE_H_
#define C2017_WPILIB_WPILIB_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/can_wrapper.h"
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
  muan::wpilib::PcmWrapper* pcm_;

  frc971::control_loops::drivetrain::InputQueue* input_queue_;
  frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  VictorSP motor_left_;
  VictorSP motor_right_;

  Encoder encoder_left_, encoder_right_;
};

class WpilibInterface {
 public:
  WpilibInterface();

  void WriteActuators();
  void ReadSensors();

 private:
  muan::wpilib::CanWrapper can_;

  muan::wpilib::gyro::GyroReader gyro_;
  DrivetrainInterface drivetrain_;
};

}  // wpilib

}  // c2017

#endif  // C2017_WPILIB_WPILIB_INTERFACE_H_
