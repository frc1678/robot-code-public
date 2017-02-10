#ifndef C2017_WPILIB_LIGHTS_INTERFACE_H_
#define C2017_WPILIB_LIGHTS_INTERFACE_H_

#include "WPILib.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace wpilib {

class LightsInterface {
 public:
  LightsInterface();
  void WriteActuators();

 private:
  DigitalOutput red_, green_, blue_;
};

}  // namespace wpilib
}  // namespace c2017

#endif  // C2017_WPILIB_LIGHTS_INTERFACE_H_
