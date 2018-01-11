#ifndef C2018_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
#define C2018_WPILIB_SUPERSTRUCTURE_INTERFACE_H_

#include "WPILib.h"
#include "muan/wpilib/pcm_wrapper.h"
#include "muan/wpilib/can_wrapper.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {
namespace wpilib {

class SuperstructureInterface {
 public:
  explicit SuperstructureInterface(muan::wpilib::CanWrapper* can_wrapper);

  void WriteActuators();
  void ReadSensors();

 private:
  // Output queue readers, motors, and sensors go here
  muan::wpilib::PdpQueue::QueueReader pdp_status_;

  muan::wpilib::PcmWrapper* pcm_;
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_SUPERSTRUCTURE_INTERFACE_H_
