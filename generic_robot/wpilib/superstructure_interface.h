#ifndef GENERIC_ROBOT_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
#define GENERIC_ROBOT_WPILIB_SUPERSTRUCTURE_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "muan/wpilib/can_wrapper.h"
#include "generic_robot/queue_manager/queue_manager.h"

namespace generic_robot {
namespace wpilib {

class SuperstructureInterface {
 public:
  explicit SuperstructureInterface(muan::wpilib::CanWrapper* can_wrapper);

  void WriteActuators();
  void ReadSensors();

 private:
  // Output queue readers, motors, and sensors go here
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>::QueueReader pdp_status_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace generic_robot

#endif  // GENERIC_ROBOT_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
