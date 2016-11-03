#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_CONTROLLER_INTAKE_CONTROLLER_H_
#define O0216_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_CONTROLLER_INTAKE_CONTROLLER_H_

#include "o2016/subsystems/superstructure/intake/controller/intake_constants.h"
#include "muan/units/units.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"

namespace frc1678{

namespace o2016 {

namespace intake {

class IntakeController {
  public: 
    IntakeController();
    ~IntakeController();
    muan::units::Voltage Update(muan::units::Angle goal, muan::units::Angle sensor_value);
    muan::units::Angle GetAngle() const;
    void SetAngle(muan::units::Angle theta);
    bool AtGoal() const;

  private:
    muan::control::StateSpaceController<1, 2, 1> controller_;
    muan::control::StateSpaceObserver<1, 2, 1> observer_;
    bool at_goal_;

    Angle angle_tolerance_;
    AngularVelocity velocity_tolerance_;
};

}

}

}



#endif 
