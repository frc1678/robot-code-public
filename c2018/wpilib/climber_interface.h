#ifndef C2018_WPILIB_CLIMBER_INTERFACE_H_
#define C2018_WPILIB_CLIMBER_INTERFACE_H_

#include <algorithm>
#include <cmath>

#include "WPILib.h"
#include "c2018/subsystems/climber/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"

namespace c2018 {
namespace wpilib {

using climber::ClimberInputProto;
using climber::ClimberOutputProto;
using climber::ClimberInputQueue;
using climber::ClimberOutputQueue;
using muan::queues::QueueManager;

class ClimberInterface {
 public:
  explicit ClimberInterface(muan::wpilib::CanWrapper* can_wrapper);
  void ReadSensors();
  void WriteActuators();

 private:
  ClimberInputQueue* input_queue_;
  ClimberOutputQueue::QueueReader output_reader_;
  muan::wpilib::PdpQueue::QueueReader pdp_reader_;

  VictorSP winch_;
  Encoder winch_encoder_;
  double winch;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_CLIMBER_INTERFACE_H_

