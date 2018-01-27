#ifndef O2017_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
#define O2017_WPILIB_SUPERSTRUCTURE_INTERFACE_H_

#include "WPILib.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "o2017/queue_manager/queue_manager.h"

namespace o2017 {
namespace wpilib {

class SuperstructureInterface {
 public:
  explicit SuperstructureInterface(muan::wpilib::PcmWrapper* pcm_);

  void WriteActuators();
  void ReadSensors();

 private:
  o2017::superstructure::InputQueue* input_queue_;
  o2017::superstructure::OutputQueue::QueueReader output_queue_;

  VictorSP climber_motor_;
  Encoder climber_encoder_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace o2017

#endif  // O2017_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
