#include "c2018/wpilib/lights_interface.h"

namespace c2018 {
namespace wpilib {

constexpr uint32_t kLights = 5;  // DIO:5

// TODO(Aakash) Test if port number is correct

LightsInterface::LightsInterface(): lights_(c2018::wpilib::kLights),
  output_reader_(muan::queues::QueueManager<LightsOutputProto>::Fetch()->MakeReader()) {}

void LightsInterface::WriteActuators() {
  LightsOutputProto output;
  if (output_reader_.ReadLastMessage(&output)) {
    lights_.Set(output->on());
  } else {
    lights_.Set(false);
  }
}

}  // namespace wpilib
}  // namespace c2018
