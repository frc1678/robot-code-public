#include "c2018/wpilib/wpilib_interface.h"
#include "muan/queues/queue_manager.h"

namespace c2018 {
namespace wpilib {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::gyro::GyroMessageProto;
using muan::wpilib::PdpMessage;

DEFINE_int32(gyro_time, 10, "How long to calibrate the gyro for.");

WpilibInterface::WpilibInterface()
    : can_{QueueManager<PdpMessage>::Fetch()},
      gyro_{QueueManager<GyroMessageProto>::Fetch(),
            QueueManager<DriverStationProto>::Fetch(), FLAGS_gyro_time, false},
      drivetrain_{&can_},
      score_{&can_} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();

  std::thread gyro_thread(std::ref(gyro_));
  gyro_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_.WriteActuators();
  score_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_.ReadSensors();
  score_.ReadSensors();
}

}  // namespace wpilib
}  // namespace c2018
