#include "o2018/interfaces/interface_runner.h"

namespace o2018 {
namespace interfaces {

void InterfaceRunner::ReadSensors() {
  arm_.ReadSensors();
  drive_.ReadSensors();
}

void InterfaceRunner::WriteActuators() {
  arm_.WriteActuators();
  drive_.WriteActuators();
}

}  // namespace interfaces
}  // namespace o2018
