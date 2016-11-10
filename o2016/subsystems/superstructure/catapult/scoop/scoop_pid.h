#ifndef SCOOP_PID_H_
#define SCOOP_PID_H_

#include "o2016/subsystems/superstructure/catapult/scoop/scoop_constants.h"
#include "muan/units/units.h"
#include "muan/control/pid_controller.h"

namespace o2016 {

namespace catapult {

using namespace ::muan::units;

class ScoopPid {
 public:
  ScoopPid();
  ~ScoopPid() = default;
  Voltage Update(Angle goal, Angle sensor_value);
  Angle get_angle() const;
  AngularVelocity get_angular_velocity() const;
  void set_angle(Angle theta);
  bool is_done() const;

 private:
  muan::PidController controller_ = muan::PidController(18., 0., 1.);
  Angle angle_;
  bool done;
  constexpr static Angle angle_tolerance = 0.1 * rad;
  constexpr static AngularVelocity velocity_tolerance = 0.1 * rad / s;
};

} // catapult

} // o2016

#endif // SCOOP_H_
