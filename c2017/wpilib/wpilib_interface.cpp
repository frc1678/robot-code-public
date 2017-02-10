#include "c2017/wpilib/wpilib_interface.h"
#include "muan/units/units.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace wpilib {

WpilibInterface::WpilibInterface()
    : can_{&QueueManager::GetInstance().pdp_status_queue()},
      gyro_{QueueManager::GetInstance().gyro_queue(), true},
      drivetrain_{},
      superstructure_{&can_},
      lights_{} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();

  std::thread gyro_thread(std::ref(gyro_));
  gyro_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_.WriteActuators();
  superstructure_.WriteActuators();
  lights_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_.ReadSensors();
  superstructure_.ReadSensors();
}

}  // namespace wpilib
}  // namespace c2017
