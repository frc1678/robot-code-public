#include "wpilib_interface.h"
#include "muan/units/units.h"
#include "generic_robot/queue_manager/queue_manager.h"

namespace generic_robot {

namespace wpilib {

namespace ports {

}  // ports

constexpr double kMaxVoltage = 4;  // 4 volt bringup voltage

WpilibInterface::WpilibInterface()
    : can_{&QueueManager::GetInstance().pdp_status_queue()} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void WpilibInterface::WriteActuators() {
}

void WpilibInterface::ReadSensors() {
}

}  // wpilib

}  // generic_robot
