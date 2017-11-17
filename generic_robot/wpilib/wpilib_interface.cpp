#include "generic_robot/wpilib/wpilib_interface.h"

namespace generic_robot {
namespace wpilib {

DEFINE_int32(gyro_time, 10, "How long to calibrate the gyro for.");

WpilibInterface::WpilibInterface()
    : can_{QueueManager::GetInstance()->pdp_status_queue()},
      gyro_{QueueManager::GetInstance()->gyro_queue(),
            QueueManager::GetInstance()->driver_station_queue(),
            FLAGS_gyro_time, true},
      drivetrain_{&can_},
      superstructure_{&can_} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_.WriteActuators();
  superstructure_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_.ReadSensors();
  superstructure_.ReadSensors();
}

}  // namespace wpilib
}  // namespace generic_robot
