#ifndef C2019_INTERFACES_INTERFACE_RUNNER_H_
#define C2019_INTERFACES_INTERFACE_RUNNER_H_

#include "c2019/interfaces/drive_interface.h"
#include "c2019/interfaces/superstructure_interface.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/can_wrapper.h"

namespace c2019 {
namespace interfaces {

class InterfaceRunner {
 public:
  InterfaceRunner();
  void ReadSensors();
  void WriteActuators();
  void operator()();

 private:
  muan::wpilib::CanWrapper can_;
  DrivetrainInterface drive_;
  SuperstructureInterface superstructure_;
};

}  // namespace interfaces
}  // namespace c2019

#endif  // C2019_INTERFACES_INTERFACE_RUNNER_H_
