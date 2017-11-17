#include "o2017/wpilib/wpilib_interface.h"

namespace o2017 {
namespace wpilib {

namespace ports {}  // namespace ports

constexpr double kMaxVoltage = 4;  // 4 volt bringup voltage

WpilibInterface::WpilibInterface() : can_{&QueueManager::GetInstance().pdp_status_queue()} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_interface_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_interface_.ReadSensors();
}

}  // namespace wpilib

}  // namespace o2017
