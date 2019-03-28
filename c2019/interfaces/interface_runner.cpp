#include "c2019/interfaces/interface_runner.h"

namespace c2019 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::wpilib::PdpMessage;

InterfaceRunner::InterfaceRunner()
    : can_(QueueManager<PdpMessage>::Fetch()), superstructure_(&can_) {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void InterfaceRunner::ReadSensors() {
  drive_.ReadSensors();
  superstructure_.ReadSensors();
}

void InterfaceRunner::WriteActuators() {
  drive_.WriteActuators();
  superstructure_.WriteActuators();
}

}  // namespace interfaces
}  // namespace c2019
