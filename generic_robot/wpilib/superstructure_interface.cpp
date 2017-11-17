#include "generic_robot/wpilib/superstructure_interface.h"

namespace generic_robot {
namespace wpilib {

namespace constants {

// Ports and sensor ratios go here

constexpr double kMaxVoltage = 12;

}  // namespace constants

SuperstructureInterface::SuperstructureInterface(muan::wpilib::CanWrapper* can_wrapper)
    : pdp_status_{QueueManager::GetInstance()->pdp_status_queue()->MakeReader()},
      pcm_{can_wrapper->pcm()} {}

void SuperstructureInterface::ReadSensors() {
}

void SuperstructureInterface::WriteActuators() {
}

}  // namespace wpilib
}  // namespace generic_robot
