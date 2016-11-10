#ifndef STOP_PID_H_
#define STOP_PID_H_

#include "muan/units/units.h"
#include "muan/control/pid_controller.h"

namespace o2016 {

namespace catapult {

using namespace ::muan::units;

class StopPid {
 public:
  StopPid();
  ~StopPid() = default;
  Voltage Update(Angle goal, Angle sensor_value);
  Angle get_angle() const;
  AngularVelocity get_angular_velocity() const;
  void set_angle(Angle theta);
  bool is_done() const;

 private:
  muan::PidController controller_ = muan::PidController(100., 0., 0.);
  Angle angle_;
  bool done;
  constexpr static Angle angle_tolerance = 0.1 * rad;
  constexpr static AngularVelocity velocity_tolerance = 0.1 * rad / s;
};

} // catapult

} // o2016

#endif // STOP_H_
