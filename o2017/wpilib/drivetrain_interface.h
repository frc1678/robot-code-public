#ifndef O2017_WPILIB_DRIVETRAIN_INTERFACE_H_
#define O2017_WPILIB_DRIVETRAIN_INTERFACE_H_

#include "WPILib.h"
#include "o2017/queue_manager/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace o2017 {
namespace wpilib {

class DrivetrainInterface {
 public:
  explicit DrivetrainInterface(muan::wpilib::PcmWrapper* pcm);

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

}  // namespace wpilib
}  // namespace o2017

#endif  // O2017_WPILIB_DRIVETRAIN_INTERFACE_H_
