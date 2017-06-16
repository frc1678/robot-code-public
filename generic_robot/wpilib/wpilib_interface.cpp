#include "generic_robot/wpilib/wpilib_interface.h"

namespace generic_robot {
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

}  // namespace generic_robot
