#ifndef C2019_INTERFACES_INTERFACE_RUNNER_H_
#define C2019_INTERFACES_INTERFACE_RUNNER_H_

#include "c2019/interfaces/drive_interface.h"

namespace c2019 {
namespace interfaces {

class InterfaceRunner {
 public:
  void ReadSensors();
  void WriteActuators();

 private:
  DrivetrainInterface drive_;
};

}  // namespace interfaces
}  // namespace c2019

#endif  // C2019_INTERFACES_INTERFACE_RUNNER_H_
