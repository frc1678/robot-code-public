#include "wpilib_interface.h"
#include "muan/units/units.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

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

}  // c2017
