#ifndef C2018_WPILIB_LIGHTS_INTERFACE_H_
#define C2018_WPILIB_LIGHTS_INTERFACE_H_

#include "WPILib.h"
#include "muan/queues/queue_manager.h"
#include "c2018/subsystems/lights/queue_types.h"

namespace c2018 {
namespace wpilib {

using lights::LightsOutputProto;
using lights::LightsOutputQueue;


class LightsInterface {
 public:
  LightsInterface();
  void WriteActuators();

 private:
  DigitalOutput lights_;
  LightsOutputQueue::QueueReader output_reader_;
};

}  // namespace wpilib
}  // namespace c2018

#endif  // C2018_WPILIB_LIGHTS_INTERFACE_H_

