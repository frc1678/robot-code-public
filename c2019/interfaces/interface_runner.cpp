#include "c2019/interfaces/interface_runner.h"

namespace c2019 {
namespace interfaces {

void InterfaceRunner::ReadSensors() { drive_.ReadSensors(); }

void InterfaceRunner::WriteActuators() { drive_.WriteActuators(); }

}  // namespace interfaces
}  // namespace c2019
