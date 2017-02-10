#include "c2017/wpilib/lights_interface.h"

namespace c2017 {

namespace wpilib {

namespace ports {

namespace lights {

constexpr uint32_t kRed = 2;
constexpr uint32_t kGreen = 5;
constexpr uint32_t kBlue = 4;
}  // namespace lights

}  // namespace ports

LightsInterface::LightsInterface()
    : red_(c2017::wpilib::ports::lights::kRed),
      green_(c2017::wpilib::ports::lights::kGreen),
      blue_(c2017::wpilib::ports::lights::kBlue) {}

void LightsInterface::WriteActuators() {
  auto lights_output = QueueManager::GetInstance().lights_output_queue().MakeReader().ReadLastMessage();
  if (lights_output) {
    red_.Set(lights_output.value()->red());
    green_.Set(lights_output.value()->green());
    blue_.Set(lights_output.value()->blue());
  }
}

}  // namespace wpilib
}  // namespace c2017
