#ifndef C2018_WPILIB_SCORE_INTERFACE_H_
#define C2018_WPILIB_SCORE_INTERFACE_H_

#include <algorithm>
#include <cmath>

#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "WPILib.h"

namespace c2018 {
namespace wpilib {

using muan::queues::QueueManager;
using score_subsystem::ScoreSubsystemInputProto;
using score_subsystem::ScoreSubsystemOutputProto;
using score_subsystem::ScoreSubsystemInputQueue;
using score_subsystem::ScoreSubsystemOutputQueue;

class ScoreSubsystemInterface {
 public:
  explicit ScoreSubsystemInterface(muan::wpilib::CanWrapper* can_wrapper);
  void WriteActuators();
  void ReadSensors();

 private:
  ScoreSubsystemInputQueue* input_queue_;
  ScoreSubsystemOutputQueue::QueueReader output_reader_;
  muan::wpilib::PdpQueue::QueueReader pdp_reader_;

  VictorSP elevator_;
  VictorSP wrist_;
  VictorSP high_roller_;
  VictorSP low_roller_;
  Encoder elevator_encoder_;
  Encoder wrist_encoder_;
  DigitalInput has_cube_;
  DigitalInput elevator_hall_;
  DigitalInput wrist_hall_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_SCORE_INTERFACE_H_

