#ifndef GENERIC_ROBOT_WPILIB_DRIVETRAIN_INTERFACE_H_
#define GENERIC_ROBOT_WPILIB_DRIVETRAIN_INTERFACE_H_

#include "WPILib.h"
#include "generic_robot/queue_manager/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace generic_robot {
namespace wpilib {

class DrivetrainInterface {
 public:
  explicit DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper);

  void WriteActuators();
  void ReadSensors();

 private:
  frc971::control_loops::drivetrain::InputQueue* input_queue_;
  frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  VictorSP motor_left_;
  VictorSP motor_right_;

  Encoder encoder_left_, encoder_right_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_DRIVETRAIN_INTERFACE_H_
