#ifndef SCOOP_H_
#define SCOOP_H_

#include "o2016/subsystems/superstructure/catapult/scoop/scoop_constants.h"
#include "muan/units/units.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"

using namespace muan::units;

class Scoop {
 public:
  Scoop();
  ~Scoop() = default;
  Voltage Update(Angle goal, Angle sensor_value);
  Angle get_angle() const;
  AngularVelocity get_angular_velocity() const;
  void set_angle(Angle theta);
  bool is_done() const;

 private:
  muan::control::StateSpaceController<1, 2, 1> controller_;
  muan::control::StateSpaceObserver<1, 2, 1> observer_;
  bool done;
  const Angle angle_tolerance = 0.1 * rad;
  const AngularVelocity velocity_tolerance = 0.1 * rad / s;
};

#endif // SCOOP_H_
