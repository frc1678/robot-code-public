#include "c2018/wpilib/superstructure_interface.h"

namespace c2018 {
namespace wpilib {

using muan::queues::QueueManager;
using muan::wpilib::PdpMessage;

namespace constants {

// Ports and sensor ratios go here

constexpr double kMaxVoltage = 12;

}  // namespace constants

SuperstructureInterface::SuperstructureInterface(muan::wpilib::CanWrapper* can_wrapper)
    : pdp_status_{QueueManager<PdpMessage>::Fetch()->MakeReader()},
      pcm_{can_wrapper->pcm()} {}

void SuperstructureInterface::ReadSensors() {
}

void SuperstructureInterface::WriteActuators() {
}

}  // namespace wpilib
}  // namespace c2018
