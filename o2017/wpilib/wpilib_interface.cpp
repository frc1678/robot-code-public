#include "o2017/wpilib/wpilib_interface.h"

namespace o2017 {
namespace wpilib {

namespace ports {}  // namespace ports

constexpr double kMaxVoltage = 4;  // 4 volt bringup voltage

WpilibInterface::WpilibInterface()
    : can_(&QueueManager::GetInstance()->pdp_status_queue()),
      gyro_(QueueManager::GetInstance()->gyro_queue(),
            &QueueManager::GetInstance()->driver_station_queue(), 10),
      drivetrain_interface_(can_.pcm()),
      superstructure_interface_(can_.pcm()) {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();

  std::thread gyro_thread(std::ref(gyro_));
  gyro_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_interface_.WriteActuators();
  superstructure_interface_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_interface_.ReadSensors();
  superstructure_interface_.ReadSensors();
}

}  // namespace wpilib

}  // namespace o2017
